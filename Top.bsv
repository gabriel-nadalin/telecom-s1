import RS232::*;
import GetPut::*;
import ThreeLevelIO::*;
import HDB3Encoder::*;
import HDB3Decoder::*;
import E1Unframer::*;
import ICMPReplier::*;
import Connectable::*;
import BUtils::*;
import PAClib::*;
import FIFOF::*;
import Vector::*;
import Clocks::*;
import SPIFlash::*;

interface Top;
    interface RS232 rs232;
    interface Reset rs232_rst;
    (* prefix="" *)
    interface SPIFlashPins flash;
    (* always_ready, prefix="", result="LED" *)
    method Bit#(6) led;
    (* prefix="" *)
    interface ThreeLevelIOPins tlio_pins;
    (* always_ready *)
    method Bool dbg2;
endinterface

(* synthesize *)
module mkTop#(Clock clk_uart, Clock clk_slow)(Top);
    Reset rst_uart <- mkAsyncResetFromCR(2, clk_uart);
    UART#(16) uart <- mkUART(8, NONE, STOP_1, 1, clocked_by clk_uart, reset_by rst_uart);
    rule discard_uart_input;
        let b <- uart.tx.get;
    endrule

    Reg#(Bit#(6)) led_reg <- mkReg(0);

    ThreeLevelIO tlio <- mkThreeLevelIO(True);
    HDB3Encoder hdb3enc <- mkHDB3Encoder;
    mkConnection(hdb3enc.out, tlio.in);

    HDB3Decoder hdb3dec <- mkHDB3Decoder;
    mkConnection(tlio.out, hdb3dec.in);

    E1Unframer unfr <- mkE1Unframer;
    mkConnection(hdb3dec.out, unfr.in);

    Reg#(Bool) ligacao <- mkReg(False);
    Bit#(9) reset_value = 9'b1_0000_0000;
    Reg#(Bit#(9)) octet <- mkReg(reset_value);
    Reg#(Bit#(8)) last_ts16 <- mkReg(0);

    rule select_desired_ts;
        match {.ts, .value} <- unfr.out.get;
        if (ts == 16) action
            let octet_aux = octet;
            octet_aux = octet >> 1 | extend(value) << 8;
            if (octet_aux[0] == 1'b0) action
                if (octet_aux[8:1] != last_ts16) begin
                    ligacao <= True;
                end
                last_ts16 <= octet_aux[8:1];
                octet_aux = reset_value;
            endaction
            octet <= octet_aux;
        endaction
    endrule

    Reg#(LBit#(1506)) frame_size <- mkReg(0);
    SyncFIFOIfc#(Bit#(8)) fifo_uart <- mkSyncFIFOFromCC(2, clk_uart);
    mkConnection(toGet(fifo_uart), uart.rx);

    Reset rst_slow <- mkAsyncResetFromCR(2, clk_slow);
    ICMPReplier icmp <- mkICMPReplier(clocked_by clk_slow, reset_by rst_slow);
    SyncFIFOIfc#(Tuple2#(Bool, Bit#(8))) icmp_in <- mkSyncFIFOFromCC(2, clk_slow);
    SyncFIFOIfc#(Tuple2#(Bool, Bit#(8))) icmp_out <- mkSyncFIFOToCC(2, clk_slow, rst_slow);
    mkConnection(toGet(icmp_in), icmp.in);
    mkConnection(icmp.out, toPut(icmp_out));

    SPIFlash spiflash <- mkSPIFlash;
    Reg#(Bit#(8)) voice_byte <- mkReg(0);
    
    // E1 framer
    Reg#(Bit#(9)) tx_index <- mkReg(0);
    rule produce_fas_nfas (tx_index[7:0] < 8);
        let i = tx_index[7:0];
        if (i == 0 && ligacao) begin
            let value <- spiflash.out.get;
            voice_byte <= value;
        end
        let fas_nfas = reverseBits(tx_index[8] == 1'b0 ? 8'b10011011 : 8'b11000111);
        hdb3enc.in.put(fas_nfas[i]);
        tx_index <= tx_index + 1;
    endrule
    rule produce_signal (tx_index[7:0] >= 128 && tx_index[7:0] < 136);
        let i = tx_index[7:0] % 8;
        hdb3enc.in.put(last_ts16[i]);
        tx_index <= tx_index + 1;
    endrule
    rule produce_voice ((tx_index[7:0] >= 8 && tx_index[7:0] < 128) || tx_index[7:0] >= 136);
        let i = tx_index[7:0] % 8;
        let sample = ligacao ? reverseBits(voice_byte) : reverseBits(8'h55);
        hdb3enc.in.put(sample[i]);
        tx_index <= tx_index + 1;
    endrule

    interface rs232 = uart.rs232;
    interface rs232_rst = rst_uart;
    interface flash = spiflash.pins;
    method led = ~led_reg;
    interface tlio_pins = tlio.pins;
    method dbg2 = tx_index == 0;
endmodule
