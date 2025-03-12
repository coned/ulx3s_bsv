import Clocks :: *;
import Vector::*;
import FIFO::*;
import BRAM::*;
import BRAMFIFO::*;
import Uart::*;
import Sdram::*;

import SimpleFloat::*;
import FloatingPoint::*;

interface PEInterface;
	method Action receiveA(Bit#(8) a_in);
	method Action receiveB(Bit#(8) b_in);
	method ActionValue#(Bit#(8)) sendA();
	method ActionValue#(Bit#(8)) sendB();
	method ActionValue#(Bit#(8)) getResult();
endinterface

module mkPE(PEInterface);
    Reg#(Bit#(8)) regA        <- mkReg(0);
    Reg#(Bit#(8)) regB        <- mkReg(0);
    Reg#(Float)    regSum      <- mkReg(0);

    Reg#(Bool)     hasA        <- mkReg(False);
    Reg#(Bool)     hasB        <- mkReg(False);
	Reg#(Bit#(2))  count       <- mkReg(0);
    Reg#(Bool)     resultReady <- mkReg(False);

    FloatTwoOp fmult <- mkFloatMult;
    FloatTwoOp fadd  <- mkFloatAdd;

   	rule multiply (hasA && hasB);
        Float a_float = unpack(regA);
        Float b_float = unpack(regB);
      	fmult.put(a_float, b_float);
      	hasA <= False; // Clear flags after using the data
      	hasB <= False;
   	endrule

   	rule accumulate;
      	let product <- fmult.get;
      	fadd.put(regSum, product);
   	endrule

   	rule updateSum;
      	let newSum <- fadd.get;
      	regSum <= newSum;
      	count <= count + 1;
		if (count == 3) begin
			resultReady <= True;
			count <= 0;
		end
   	endrule

	method Action receiveA (Bit#(8) a_in) if (!hasA);
        regA <= a_in;
      	hasA <= True;
   	endmethod

   	method Action receiveB(Bit#(8) b_in) if (!hasB);
      	regB <= b_in;
      	hasB <= True;
   	endmethod

   	method ActionValue#(Bit#(8)) sendA() if (hasA);
      	return regA;
   	endmethod

   	method ActionValue#(Bit#(8)) sendB() if (hasB);
      	return regB;
   	endmethod

    method ActionValue#(Bit#(8)) getResult() if (resultReady);
        resultReady <= False;
        return pack(regSum);
    endmethod
endmodule

interface MatrixMultiplierInterface;
	method Action loadMatrixA(Vector#(4, Vector#(4, Bit#(8))) matrixA);
	method Action loadMatrixB(Vector#(4, Vector#(4, Bit#(8))) matrixB);
	method ActionValue#(Vector#(4, Vector#(4, Bit#(8)))) getResult();
endinterface

module mkSystolicArray(MatrixMultiplierInterface);

	PEInterface pes[4][4];
	for (Integer i = 0; i < 4; i = i + 1) begin
		for (Integer j = 0; j < 4; j = j + 1) begin
			pes[i][j] <- mkPE;
		end
	end

	FIFO#(Bit#(32)) fifoA[4][5]; // Additional column for input
	FIFO#(Bit#(32)) fifoB[5][4]; // Additional row for input

	for (Integer i = 0; i < 4; i = i + 1) begin
		for (Integer j = 0; j < 5; j = j + 1) begin
			fifoA[i][j] <- mkFIFO;
		end
	end
	for (Integer i = 0; i < 5; i = i + 1) begin
		for (Integer j = 0; j < 4; j = j + 1) begin
			fifoB[i][j] <- mkFIFO;
		end
	end

	// Rules to pass A and B through the array
	for (Integer i = 0; i < 4; i = i + 1) begin
		for (Integer j = 0; j < 4; j = j + 1) begin
			rule passA;
				fifoA[i][j].deq;
				pes[i][j].receiveA(fifoA[i][j].first);
				fifoA[i][j+1].enq(fifoA[i][j].first);
			endrule

			// Pass B to the PE and to the next FIFO
			rule passB;
				fifoB[i][j].deq;
				pes[i][j].receiveB(fifoB[i][j].first);
				fifoB[i+1][j].enq(fifoB[i][j].first);
			endrule
		end
	end

	method Action loadMatrixA(Vector#(4, Vector#(4, Bit#(8))) matrixA);
		// Enqueue matrix A elements into fifoA[i][0]
		for (Integer i = 0; i < 4; i = i + 1) begin
			for (Integer k = 0; k < 4; k = k + 1) begin
				fifoA[i][0].enq(matrixA[i][k]);
			end
		end
	endmethod

	method Action loadMatrixB(Vector#(4, Vector#(4, Bit#(8))) matrixB);
		// Enqueue matrix B elements into fifoB[0][j]
		for (Integer k = 0; k < 4; k = k + 1) begin
			for (Integer j = 0; j < 4; j = j + 1) begin
				fifoB[0][j].enq(matrixB[k][j]);
			end
		end
	endmethod


	method ActionValue#(Vector#(4, Vector#(4, Bit#(8)))) getResult();
		Vector#(4, Vector#(4, Bit#(8))) result = replicate(replicate(0));
		for (Integer i = 0; i < 4; i = i + 1) begin
			for (Integer j = 0; j < 4; j = j + 1) begin
				let val <- pes[i][j].getResult();
				result[i][j] = val;
			end
		end
		return result;
	endmethod

endmodule

interface HwMainIfc;
	method ActionValue#(Bit#(8)) serial_tx;
	method Action serial_rx(Bit#(8) rx);
endinterface

typedef enum {IDLE, LOADING, PROCESSING, OUTPUTTING} State deriving (Eq, Bits);

module mkHwMain#(Ulx3sSdramUserIfc mem) (HwMainIfc);
    Clock curclk <- exposeCurrentClock;
    Reset currst <- exposeCurrentReset;

    Reg#(Bit#(32)) cycles <- mkReg(0);
    rule incCyclecount;
        cycles <= cycles + 1;
    endrule

    Reg#(Bit#(32)) processingStartCycle <- mkReg(0);
    Reg#(Bit#(5)) inputEnqueued <- mkReg(0);

    FIFO#(Bit#(32)) inputAQ <- mkSizedBRAMFIFO(32);
    FIFO#(Bit#(32)) inputBQ <- mkSizedBRAMFIFO(32);
    Reg#(Vector#(4, Vector#(4, Bit#(8)))) matrixA_bits <- mkReg(replicate(replicate(0)));
    Reg#(Vector#(4, Vector#(4, Bit#(8)))) matrixB_bits <- mkReg(replicate(replicate(0)));

    MatrixMultiplierInterface systolicArray <- mkSystolicArray();

    Reg#(Bit#(5)) loadMatrixCnt <- mkReg(0);
    Reg#(Bit#(2)) loadMatrixCol <- mkReg(0);
    Reg#(Bit#(2)) loadMatrixRow <- mkReg(0);
    rule loadMatrix (loadMatrixCnt < 16);
        inputAQ.deq;
        inputBQ.deq;

        Vector#(4, Bit#(8)) rowA = matrixA_bits[loadMatrixRow];
        Vector#(4, Bit#(8)) rowB = matrixB_bits[loadMatrixRow];

        rowA = inputAQ.first;
        rowB = inputBQ.first;

        matrixA_bits <= rowA;
        matrixB_bits <= rowB;

        loadMatrixCnt <= loadMatrixCnt + 1;
        loadMatrixCol <= loadMatrixCol + 1;
        if (loadMatrixCol == 3) begin
            loadMatrixRow <= loadMatrixRow + 1;
            loadMatrixCol <= 0;
        end

        if (loadMatrixCnt == 16) begin
            systolicArray.loadMatrixA(matrixA_bits);
            systolicArray.loadMatrixB(matrixB_bits);
            loadMatrixCnt <= 0;
            loadMatrixRow <= 0;
            loadMatrixCol <= 0;
            processingStartCycle <= cycles;
        end
    endrule

    Reg#(Bool) computationDone <- mkReg(False);
	Reg#(Vector#(4, Vector#(4, Bit#(8)))) resultMatrix <- mkReg(replicate(replicate(0)));
    rule getSystolicResult (!computationDone);
        let result <- systolicArray.getResult();
        resultMatrix <= result;
        computationDone <= True;
        $write("Processing done in %d cycles\n", cycles - processingStartCycle);
    endrule

	Reg#(Vector#(4,Bit#(8))) outputDeSerializer <- mkReg(?);
	Reg#(Vector#(4, Bit#(8))) inputDeSerializer <- mkReg(?);
    Reg#(Bit#(2)) inputDeSerializerIdx <- mkReg(0);

    method ActionValue#(Bit#(8)) serial_tx;
		Bit#(8) ret = resultMatrix[loadMatrixRow][loadMatrixCol];
		if (loadMatrixCol == 3 && loadMatrixRow == 3) begin
			loadMatrixRow <= 0;
			loadMatrixCol <= 0;
		end else if (loadMatrixCol == 3) begin
			loadMatrixRow <= loadMatrixRow + 1;
			loadMatrixCol <= 0;
		end else begin
			loadMatrixCol <= loadMatrixCol + 1;
		end
		return ret;
    endmethod

    method Action serial_rx(Bit#(8) d);
        Vector#(4, Bit#(8)) des_value = inputDeSerializer;
        des_value[inputDeSerializerIdx] = d;
        inputDeSerializerIdx <= inputDeSerializerIdx + 1;
        inputDeSerializer <= des_value;

        if (inputDeSerializerIdx == 3) begin
            if (inputEnqueued < 16) begin
                inputAQ.enq(pack(des_value));
            end else begin
                inputBQ.enq(pack(des_value));
            end
            inputEnqueued <= inputEnqueued + 1;
            inputDeSerializerIdx <= 0;

            if (inputEnqueued == 31) begin
				processingStartCycle <= cycles;
                $write("All inputs received at cycle %d\n", cycles);
            end
        end
    endmethod
endmodule