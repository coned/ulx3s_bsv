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
	method Action receiveA(Bit#(32) a_in);
	method Action receiveB(Bit#(32) b_in);
	method ActionValue#(Bit#(32)) sendA();
	method ActionValue#(Bit#(32)) sendB();
	method ActionValue#(Bit#(32)) getResult();
endinterface

module mkPE(PEInterface);
    Reg#(Bit#(32)) regA        <- mkReg(0);
    Reg#(Bit#(32)) regB        <- mkReg(0);
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

	method Action receiveA (Bit#(32) a_in) if (!hasA);
        regA <= a_in;
      	hasA <= True;
   	endmethod

   	method Action receiveB(Bit#(32) b_in) if (!hasB);
      	regB <= b_in;
      	hasB <= True;
   	endmethod

   	method ActionValue#(Bit#(32)) sendA() if (hasA);
      	return regA;
   	endmethod

   	method ActionValue#(Bit#(32)) sendB() if (hasB);
      	return regB;
   	endmethod

    method ActionValue#(Bit#(32)) getResult() if (resultReady);
        resultReady <= False;
        return pack(regSum);
    endmethod
endmodule

interface MatrixMultiplierInterface;
	method Action loadMatrixA(Vector#(4, Vector#(4, Bit#(32))) matrixA);
	method Action loadMatrixB(Vector#(4, Vector#(4, Bit#(32))) matrixB);
	method ActionValue#(Vector#(4, Vector#(4, Bit#(32)))) getResult();
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

	method Action loadMatrixA(Vector#(4, Vector#(4, Bit#(32))) matrixA);
		// Enqueue matrix A elements into fifoA[i][0]
		for (Integer i = 0; i < 4; i = i + 1) begin
			for (Integer k = 0; k < 4; k = k + 1) begin
				fifoA[i][0].enq(matrixA[i][k]);
			end
		end
	endmethod

	method Action loadMatrixB(Vector#(4, Vector#(4, Bit#(32))) matrixB);
		// Enqueue matrix B elements into fifoB[0][j]
		for (Integer k = 0; k < 4; k = k + 1) begin
			for (Integer j = 0; j < 4; j = j + 1) begin
				fifoB[0][j].enq(matrixB[k][j]);
			end
		end
	endmethod


	method ActionValue#(Vector#(4, Vector#(4, Bit#(32)))) getResult();
		Vector#(4, Vector#(4, Bit#(32))) result = replicate(replicate(0));
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
	FIFO#(Bit#(32)) outputQ <- mkSizedBRAMFIFO(32);

	Reg#(Vector#(4,Vector#(4,Float))) matrixA <- mkReg(replicate(replicate(0)));
	Reg#(Vector#(4,Vector#(4,Float))) matrixB <- mkReg(replicate(replicate(0)));
	Reg#(Vector#(4,Vector#(4,Float))) matrixC <- mkReg(replicate(replicate(0)));
	
	FloatTwoOp fmult <- mkFloatMult;
	FloatTwoOp fadd <- mkFloatAdd;

	Reg#(Bit#(5)) loadMatrixCnt <- mkReg(0);
	Reg#(Bit#(2)) loadMatrixCol <- mkReg(0);
	Reg#(Bit#(2)) loadMatrixRow <- mkReg(0);
	rule loadMatrix (  loadMatrixCnt < 16 );
		inputAQ.deq;
		inputBQ.deq;
		matrixA[loadMatrixRow][loadMatrixCol] <= unpack(inputAQ.first);
		matrixB[loadMatrixRow][loadMatrixCol] <= unpack(inputBQ.first);
		loadMatrixCnt <= loadMatrixCnt + 1;
		loadMatrixCol <= loadMatrixCol + 1;
		if ( loadMatrixCol == 3 ) begin
			loadMatrixRow <= loadMatrixRow + 1;
		end
	endrule

	Reg#(Bit#(2)) procMatrixI <- mkReg(0);
	Reg#(Bit#(2)) procMatrixJ <- mkReg(0);
	Reg#(Bit#(2)) procMatrixK <- mkReg(0);
	Reg#(Bit#(8)) fMultReqCnt <- mkReg(0);
	rule procMatrixMult ( loadMatrixCnt == 16 );
		Float av = matrixA[procMatrixI][procMatrixK];
		Float bv = matrixB[procMatrixJ][procMatrixK];

		fmult.put(av,bv);
		fMultReqCnt <= fMultReqCnt + 1;
		//$write( ">> %d %d %d ---- %x %x -- %d\n", procMatrixI, procMatrixJ, procMatrixK, av, bv, fMultReqCnt);

		procMatrixK <= procMatrixK + 1;
		if ( procMatrixK == 3 ) begin
			procMatrixJ <= procMatrixJ + 1;
			if ( procMatrixJ == 3 ) begin
				procMatrixI <= procMatrixI + 1;
				if ( procMatrixI == 3 ) begin
					loadMatrixCnt <= 0;
					//$write("Proc done!!!!!\n");
				end
			end
		end
	endrule

	FIFO#(Float) fMultQ <- mkFIFO;
	Reg#(Bit#(8)) fMultCnt <- mkReg(0);
	rule relayMultRes;
		let v <- fmult.get;
		fMultQ.enq(v);
		fMultCnt <= fMultCnt + 1;
	endrule
	
	Reg#(Bit#(2)) accumulateMatrixI <- mkReg(0);
	Reg#(Bit#(2)) accumulateMatrixJ <- mkReg(0);
	Reg#(Bit#(2)) accumulateMatrixK <- mkReg(0);
	FIFO#(Tuple3#(Bit#(2),Bit#(2), Bool)) matrixAccumulateDestQ <- mkFIFO;
	Reg#(Bool) accumulateInflight <- mkReg(False);
	Reg#(Bool) accumulatorWaitForFlush <- mkReg(False);
	Reg#(Bit#(8)) accumulateCnt <- mkReg(0);
	rule procMatrixAdd ( !accumulateInflight && !accumulatorWaitForFlush );
		//let mv <- fmult.get;
		let mv = fMultQ.first;
		fMultQ.deq;

		let cv = matrixC[accumulateMatrixI][accumulateMatrixJ];

		if ( accumulateMatrixK == 0 ) begin
			fadd.put(mv,0);
		end else begin
			fadd.put(mv,cv);
		end
		accumulateInflight <= True;
		
		Bool islast = False;
		accumulateCnt <= accumulateCnt + 1;

		accumulateMatrixK <= accumulateMatrixK + 1;
		if ( accumulateMatrixK == 3 ) begin
			accumulateMatrixJ <= accumulateMatrixJ + 1;
			if ( accumulateMatrixJ == 3 ) begin
				accumulateMatrixI <= accumulateMatrixI + 1;

				if ( accumulateMatrixI == 3 ) begin
					accumulatorWaitForFlush <= True;
					islast = True;
					$write( "Acceleration done! %d cycles\n", cycles - processingStartCycle );
				end
			end
		end
		
		//$write( "<< %d %d %d -- %d %x\n", accumulateMatrixI, accumulateMatrixJ, accumulateMatrixK, accumulateCnt, mv);
		
		matrixAccumulateDestQ.enq(tuple3(accumulateMatrixI,accumulateMatrixJ, islast));
	endrule

	
	Reg#(Bit#(8)) accumulateResCnt <- mkReg(0);
	Reg#(Bool) startOutputFlush <- mkReg(False);
	rule recvMatrixAdd (accumulateInflight);
		let av <- fadd.get;
		//Float av = unpack(32'h3ecccccd);
		let dst = matrixAccumulateDestQ.first;
		matrixAccumulateDestQ.deq;

		if ( tpl_3(dst) ) begin
			startOutputFlush <= True;
		end

		matrixC[tpl_1(dst)][tpl_2(dst)] <= av;
		
		accumulateInflight <= False;
		accumulateResCnt <= accumulateResCnt + 1;
		
		//$write( "++ %d %d %s -- %d %x\n", tpl_1(dst), tpl_2(dst), tpl_3(dst)?"True":"False", accumulateResCnt, pack(av));
	endrule

	
	FIFO#(Bit#(32)) flushOutQ <- mkFIFO;
	Reg#(Bit#(2)) flushOutputMatrixI <- mkReg(0);
	Reg#(Bit#(2)) flushOutputMatrixJ <- mkReg(0);
	rule flushOutput ( startOutputFlush );
		outputQ.enq(pack(matrixC[flushOutputMatrixI][flushOutputMatrixJ]));
		
		flushOutputMatrixJ <= flushOutputMatrixJ + 1;
		if ( flushOutputMatrixJ == 3 ) begin
			flushOutputMatrixI <= flushOutputMatrixI + 1;
			if ( flushOutputMatrixI == 3 ) begin
				startOutputFlush <= False;
				accumulatorWaitForFlush <= False;
			end
		end
	endrule






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
