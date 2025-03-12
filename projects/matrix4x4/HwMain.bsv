import Clocks :: *;
import Vector::*;
import FIFO::*;
import BRAM::*;
import BRAMFIFO::*;
import Uart::*;
import Sdram::*;

import SimpleFloat::*;
import FloatingPoint::*;

interface PEIfc;
	method Action receiveA(Bit#(32) a_in);
	method Action receiveB(Bit#(32) b_in);
	method ActionValue#(Bit#(32)) sendA();
	method ActionValue#(Bit#(32)) sendB();
	method ActionValue#(Bit#(32)) getResult();
endinterface

module mkPE(PEIfc);
    Reg#(Bit#(32)) regA        <- mkReg(0);
    Reg#(Bit#(32)) regB        <- mkReg(0);
    Reg#(Float)    regSum      <- mkReg(0);

    Reg#(Bool)     hasA        <- mkReg(False); // True if A is received and not yet multiplied
    Reg#(Bool)     hasB        <- mkReg(False);
	Reg#(Bool)     isSendA     <- mkReg(True); // True if A is sent
	Reg#(Bool)     isSendB     <- mkReg(True);
	Reg#(Bit#(3))  count       <- mkReg(0);
    Reg#(Bool)     resultReady <- mkReg(False);

    FloatTwoOp fmult <- mkFloatMult;
    FloatTwoOp fadd  <- mkFloatAdd;

   	rule multiply (hasA && hasB);
        Float a_float = unpack(regA);
        Float b_float = unpack(regB);
      	fmult.put(a_float, b_float);
      	hasA <= False;
      	hasB <= False;
		// $write( "Multiplying A = %x, B = %x\n", regA, regB );
   	endrule

   	rule accumulate;
      	let product <- fmult.get;
      	fadd.put(regSum, product);
		// $write( "Accumulating product = %x\n", product );
   	endrule

   	rule updateSum (!resultReady);
      	let newSum <- fadd.get;
      	regSum <= newSum;
		// $write( "New sum = %x, new count = %d\n", newSum, count + 1 );

      	count <= count + 1;
		if (count == 4) begin
			resultReady <= True;
		end
   	endrule

	method Action receiveA (Bit#(32) a_in) if (!hasA && isSendA);
        regA <= a_in;
      	hasA <= True;
		isSendA <= False;
   	endmethod

   	method Action receiveB(Bit#(32) b_in) if (!hasB && isSendB);
      	regB <= b_in;
      	hasB <= True;
		isSendB <= False;
   	endmethod

   	method ActionValue#(Bit#(32)) sendA() if (!isSendA);
		isSendA <= True;
      	return regA;
   	endmethod

   	method ActionValue#(Bit#(32)) sendB() if (!isSendB);
		isSendB <= True;
      	return regB;
   	endmethod

    method ActionValue#(Bit#(32)) getResult() if (resultReady);
        return pack(regSum);
    endmethod
endmodule

interface HwMainIfc;
    method Action serial_rx(Bit#(8) data_in);
    method ActionValue#(Bit#(8)) serial_tx();
endinterface

module mkHwMain#(Ulx3sSdramUserIfc mem) (HwMainIfc);
	Clock curclk <- exposeCurrentClock;
	Reset currst <- exposeCurrentReset;

	Reg#(Bit#(32)) cycles <- mkReg(0);
	rule incCyclecount;
		cycles <= cycles + 1;
	endrule

	Reg#(Bit#(32)) processingStartCycle <- mkReg(0);
	FIFO#(Bit#(32)) inputAQ <- mkSizedBRAMFIFO(32);
	FIFO#(Bit#(32)) inputBQ <- mkSizedBRAMFIFO(32);
	FIFO#(Bit#(32)) outputQ <- mkSizedBRAMFIFO(32);

	Vector#(4, FIFO#(Bit#(32))) matrixA <- replicateM(mkSizedBRAMFIFO(32));
	Vector#(4, FIFO#(Bit#(32))) matrixB <- replicateM(mkSizedBRAMFIFO(32));

	Reg#(Bit#(3)) loadMatrixARow <- mkReg(0);
	Reg#(Bit#(3)) loadMatrixBRow <- mkReg(0);
	Reg#(Bit#(3)) loadMatrixACol <- mkReg(0);
	Reg#(Bit#(3)) loadMatrixBCol <- mkReg(0);

	rule loadMatirxA (loadMatrixARow < 4 && loadMatrixACol < 4);
		inputAQ.deq;
		matrixA[loadMatrixARow].enq(unpack(inputAQ.first));
		// $write( "Loading A[%d][%d] = %x\n", loadMatrixARow, loadMatrixACol, inputAQ.first );
		if (loadMatrixARow == 3 && loadMatrixACol == 3) begin
			// Finished loading matrix A
		end else if (loadMatrixACol == 3) begin
			loadMatrixARow <= loadMatrixARow + 1;
			loadMatrixACol <= 0;
		end else begin
			loadMatrixACol <= loadMatrixACol + 1;
		end
	endrule

	rule loadMatirxB (loadMatrixBRow < 4 && loadMatrixBCol < 4);
		inputBQ.deq;
		matrixB[loadMatrixBCol].enq(unpack(inputBQ.first));
		// $write( "Loading B[%d][%d] = %x\n", loadMatrixBRow, loadMatrixBCol, inputBQ.first );
		if (loadMatrixBRow == 3 && loadMatrixBCol == 3) begin
			// Finished loading matrix B
		end else if (loadMatrixBCol == 3) begin
			loadMatrixBRow <= loadMatrixBRow + 1;
			loadMatrixBCol <= 0;
		end else begin
			loadMatrixBCol <= loadMatrixBCol + 1;
		end
	endrule

	PEIfc pes[4][4];
	for (Integer i = 0; i < 4; i = i + 1) begin
		for (Integer j = 0; j < 4; j = j + 1) begin
			pes[i][j] <- mkPE;
		end
	end

	for (Integer i = 0; i < 4; i = i + 1) begin
		rule sendMatrixA;
			matrixA[i].deq;
			pes[i][0].receiveA(pack(matrixA[i].first));
			$write( "A [%d][          ?] -> PE[%d][          0]\n", i, i);
		endrule

		rule sendMatrixB;
			matrixB[i].deq;
			pes[0][i].receiveB(pack(matrixB[i].first));
			$write( "B [          ?][%d] -> PE[          0][%d]\n", i, i);
		endrule
	end

	for (Integer i = 0; i < 4; i = i + 1) begin
		for (Integer j = 0; j + 1< 4; j = j + 1) begin
			rule sendAToPE;
				let a <- pes[i][j].sendA();
				pes[i][j + 1].receiveA(a);
				$write( "PE[%d][%d] -> PE[%d][%d]\n", i, j, i, j + 1 );
			endrule
		end
	end

	for (Integer i = 0; i + 1 < 4; i = i + 1) begin
		for (Integer j = 0; j < 4; j = j + 1) begin
			rule sendBToPE;
				let b <- pes[i][j].sendB();
				pes[i + 1][j].receiveB(b);
				$write( "PE[%d][%d] -> PE[%d][%d]\n", i, j, i + 1, j );
			endrule
		end
	end


	rule finishAcceleration;
		let r <- pes[3][3].getResult();
		$write( "Acceleration done! %d cycles\n", cycles - processingStartCycle );
	endrule

	FIFO#(Bit#(32)) flushOutQ <- mkFIFO;
	Reg#(Bool) isFlushFinished <- mkReg(False);
	Reg#(Bit#(2)) flushOutputMatrixI <- mkReg(0);
	Reg#(Bit#(2)) flushOutputMatrixJ <- mkReg(0);
	rule flushOutput ( !isFlushFinished );
		let r <- pes[flushOutputMatrixI][flushOutputMatrixJ].getResult();
		outputQ.enq(pack(r));
		//$write( "Flushing result[%d][%d] = %x\n", flushOutputMatrixI, flushOutputMatrixJ, r );
		
		if (flushOutputMatrixJ == 3 && flushOutputMatrixI == 3) begin
			flushOutputMatrixI <= 0;
			flushOutputMatrixJ <= 0;
			isFlushFinished <= True;
		end else if (flushOutputMatrixJ == 3) begin
			flushOutputMatrixI <= flushOutputMatrixI + 1;
			flushOutputMatrixJ <= 0;
		end else begin
			flushOutputMatrixJ <= flushOutputMatrixJ + 1;
		end
	endrule

	Reg#(Bit#(5)) inputEnqueued <- mkReg(0);

	Reg#(Vector#(4,Bit#(8))) outputDeSerializer <- mkReg(?);
	Reg#(Bit#(2)) outputDeSerializerIdx <- mkReg(0);
	
	Reg#(Vector#(4,Bit#(8))) inputDeSerializer <- mkReg(?);
	Reg#(Bit#(2)) inputDeSerializerIdx <- mkReg(0);

	method ActionValue#(Bit#(8)) serial_tx;
		Bit#(8) ret = 0;
		if ( outputDeSerializerIdx == 0 ) begin
			outputQ.deq;
			Vector#(4,Bit#(8)) ser_value = unpack(outputQ.first);

			outputDeSerializer <= ser_value;
			ret = ser_value[0];
		end else begin
			ret = outputDeSerializer[outputDeSerializerIdx];
		end
		outputDeSerializerIdx <= outputDeSerializerIdx + 1;
		return ret;
	endmethod

	method Action serial_rx(Bit#(8) d);
		Vector#(4,Bit#(8)) des_value = inputDeSerializer;
		des_value[inputDeSerializerIdx] = d;
		inputDeSerializerIdx <= inputDeSerializerIdx + 1;
		inputDeSerializer <= des_value;


		if (inputDeSerializerIdx == 3 ) begin
			// How is input being split to A and B correctly, even when there is more than 32 inputs?
			if ( inputEnqueued < 16 ) begin
				inputAQ.enq(pack(des_value));
			end else begin
				inputBQ.enq(pack(des_value));
			end
			inputEnqueued <= inputEnqueued + 1;

			if (inputEnqueued == 31 ) begin
				processingStartCycle <= cycles;
			end
		end
	endmethod
endmodule
