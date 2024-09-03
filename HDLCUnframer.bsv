import GetPut::*;
import FIFOF::*;
import Assert::*;

interface HDLCUnframer;
    interface Put#(Bit#(1)) in;
    interface Get#(Tuple2#(Bool, Bit#(8))) out;
endinterface

module mkHDLCUnframer(HDLCUnframer);
    // Sugestão de elementos de estado (podem ser alterados caso conveniente)
    FIFOF#(Tuple2#(Bool, Bit#(8))) fifo_out <- mkFIFOF;
    Reg#(Bool) start_of_frame <- mkReg(True);
    Bit#(9) octet_reset_value = 9'b1_0000_0000;
    Reg#(Bit#(9)) octet <- mkReg(octet_reset_value);
    Reg#(Bit#(7)) recent_bits <- mkReg(0);

    interface out = toGet(fifo_out);

    interface Put in;
        method Action put(Bit#(1) b);
            // TODO: preencha aqui com a sua lógica
            let octet_aux = octet >> 1 | extend(b) << 8;
            let recent_bits_aux = recent_bits << 1 | extend(b);
            if (start_of_frame) begin
                if (recent_bits_aux == 7'b1111110) begin
                    octet_aux = octet_reset_value;
                end else if (recent_bits_aux[5:0] == 6'b111110) begin
                    octet_aux = octet;
                end
                if (octet_aux[0] == 1'b1) begin
                    fifo_out.enq(tuple2(start_of_frame, octet_aux[8:1]));
                    octet_aux = octet_reset_value;
                    start_of_frame <= False;
                end
            end else begin
                if (recent_bits_aux == 7'b1111110) begin
                    octet_aux = octet_reset_value;
                    start_of_frame <= True;
                end else if (recent_bits_aux[5:0] == 6'b111110) begin
                    octet_aux = octet;
                end
                if (octet_aux[0] == 1'b1) begin
                    fifo_out.enq(tuple2(start_of_frame, octet_aux[8:1]));
                    octet_aux = octet_reset_value;
                end
            end
            octet <= octet_aux;
            recent_bits <= recent_bits_aux;
        endmethod
    endinterface
endmodule
