import StmtFSM::*;
import GetPut::*;
import FIFOF::*;

interface SPIFlash;
    interface Get#(Bit#(8)) out;
    interface SPIFlashPins pins;
endinterface

interface SPIFlashPins;
    (* always_ready *)
    method Bit#(1) flashClk;
    (* always_ready, always_enabled, prefix="" *)
    method Action putFlashMiso(Bit#(1) flashMiso);
    (* always_ready *)
    method Bit#(1) flashMosi;
    (* always_ready *)
    method Bit#(1) flashCs;
endinterface

module mkSPIFlash(SPIFlash);
    Reg#(Bit#(1)) clk <- mkReg(0);
    Reg#(Bit#(1)) miso <- mkRegU;
    Reg#(Bit#(1)) mosi <- mkRegU;
    Reg#(Bit#(1)) cs <- mkReg(1);

    Reg#(Bit#(4)) i <- mkRegU;
    Reg#(Bit#(8)) received <- mkRegU;
    FIFOF#(Bit#(8)) fifo_out <- mkFIFOF;

    function send(octet) = seq
        for (i <= 0; i < 8; i <= i + 1) seq
            mosi <= reverseBits(octet)[i];
            clk <= 0;
            clk <= 1;
        endseq
    endseq;

    function recv = seq
        for (i <= 0; i < 8; i <= i + 1) seq
            clk <= 0;
            clk <= 1;
            received <= (received << 1) | extend(miso);
        endseq
    endseq;

    mkAutoFSM(seq
        delay(8192);  // wait flash startup

        cs <= 0;

        send(8'h03); // READ
        send(8'h00); // addr2
        send(8'h00); // addr1
        send(8'h00); // addr0

        while (True) seq
            recv;
            fifo_out.enq(received);
        endseq
    endseq);

    interface SPIFlashPins pins;
        method flashClk = clk;
        method putFlashMiso = miso._write;
        method flashMosi = mosi;
        method flashCs = cs;
    endinterface

    interface out = toGet(fifo_out);
endmodule