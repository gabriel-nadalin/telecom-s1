import GetPut::*;
import Connectable::*;
import FIFOF::*;
import SpecialFIFOs::*;
import Vector::*;
import Assert::*;
import ThreeLevelIO::*;

interface HDB3Decoder;
    interface Put#(Symbol) in;
    interface Get#(Bit#(1)) out;
endinterface

typedef enum {
    IDLE_OR_S1,
    S2,
    S3,
    S4
} State deriving (Bits, Eq, FShow);

module mkHDB3Decoder(HDB3Decoder);
    // Sugestão de elementos de estado (podem ser alterados caso conveniente)
    Vector#(4, FIFOF#(Symbol)) fifos <- replicateM(mkPipelineFIFOF);
    Reg#(Bool) last_pulse_p <- mkReg(False);
    Reg#(State) state <- mkReg(IDLE_OR_S1);

    for (Integer i = 0; i < 3; i = i + 1)
        mkConnection(toGet(fifos[i+1]), toPut(fifos[i]));

    interface in = toPut(fifos[3]);

    interface Get out;
        method ActionValue#(Bit#(1)) get;
            let recent_symbols = tuple4(fifos[0].first, fifos[1].first, fifos[2].first, fifos[3].first);
            let value = 0;

            // TODO: preencha aqui com a sua lógica
            case (state)
                IDLE_OR_S1:
                    if (recent_symbols == tuple4(P, Z, Z, P) ||
                        recent_symbols == tuple4(N, Z, Z, N) ||
                        recent_symbols == tuple4(Z, Z, Z, P) && last_pulse_p ||
                        recent_symbols == tuple4(Z, Z, Z, N) && !last_pulse_p) action

                        if (tpl_1(recent_symbols) != Z) action
                            last_pulse_p <= !last_pulse_p;
                        endaction
                        state <= S2;
                    endaction else
                    if (tpl_1(recent_symbols) != Z) action
                        // AMI-like decoding
                        value = 1;
                        last_pulse_p <= !last_pulse_p;
                    endaction
                S2:
                    action
                        dynamicAssert(tpl_1(recent_symbols) == Z, "unexpected value on S2");
                        state <= S3;
                    endaction
                S3:
                    action
                        dynamicAssert(tpl_1(recent_symbols) == Z, "unexpected value on S3");
                        state <= S4;
                    endaction
                S4:
                    action
                        dynamicAssert(tpl_1(recent_symbols) != Z, "unexpected value on S4");
                        state <= IDLE_OR_S1;
                    endaction
            endcase

            fifos[0].deq;
            return value;
        endmethod
    endinterface
endmodule
