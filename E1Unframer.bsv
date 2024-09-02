import GetPut::*;
import FIFOF::*;
import Assert::*;

typedef Bit#(TLog#(32)) Timeslot;

interface E1Unframer;
    interface Put#(Bit#(1)) in;
    interface Get#(Tuple2#(Timeslot, Bit#(1))) out;
endinterface

typedef enum {
    UNSYNCED,
    FIRST_FAS,
    FIRST_NFAS,
    SYNCED
} State deriving (Bits, Eq, FShow);

module mkE1Unframer(E1Unframer);
    // Sugestão de elementos de estado (podem ser alterados caso conveniente)
    FIFOF#(Tuple2#(Timeslot, Bit#(1))) fifo_out <- mkFIFOF;
    Reg#(State) state <- mkReg(UNSYNCED);
    Reg#(Bit#(TLog#(8))) cur_bit <- mkRegU;
    Reg#(Timeslot) cur_ts <- mkRegU;
    Reg#(Bool) fas_turn <- mkRegU;
    Reg#(Bit#(8)) cur_byte <- mkReg(0);

    interface out = toGet(fifo_out);

    interface Put in;
        method Action put(Bit#(1) b);
            // TODO: preencha aqui com a sua lógica
            cur_byte <= (cur_byte << 1) | zeroExtend(b);
            case (state)
                UNSYNCED:
                    action
                        if (cur_byte[6:0] == 7'b0011011) begin
                            cur_bit <= 0;
                            cur_ts <= 1;
                            state <= FIRST_FAS;
                        end
                    endaction
                FIRST_FAS:
                    action
                        if (cur_bit == 7) begin
                            if (cur_ts == 0) begin
                                state <= cur_byte[6] == 1'b1 ? FIRST_NFAS : UNSYNCED;
                            end
                            cur_ts <= cur_ts + 1;
                        end
                        cur_bit <= cur_bit + 1;
                    endaction
                FIRST_NFAS:
                    action
                        if (cur_bit == 7) begin
                            if (cur_ts == 0) begin
                                state <= cur_byte[6:0] == 7'b0011011 ? SYNCED : UNSYNCED;
                                fas_turn <= False;
                            end 
                            cur_ts <= cur_ts + 1;
                        end
                        cur_bit <= cur_bit + 1;
                    endaction
                SYNCED:
                    action
                        if (cur_bit == 7) begin
                            if (cur_ts == 0) begin
                                if ((fas_turn && cur_byte[6:0] != 7'b0011011) || (!fas_turn && cur_byte[6] != 1'b1)) begin
                                    state <= UNSYNCED;
                                end
                                fas_turn <= !fas_turn;
                            end
                            cur_ts <= cur_ts + 1;
                        end
                        cur_bit <= cur_bit + 1;
                        fifo_out.enq(tuple2(cur_ts, cur_byte[0]));
                    endaction
            endcase
        endmethod
    endinterface
endmodule
