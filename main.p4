/* -*- P4_16 -*- */
#include <core.p4>
#include <v1model.p4>
#include "headers.p4"

#define MEMORY_UNIT_BIT 32
#define MEMORY_MAX_SIZE 2048

/** INSTRUCTION START **/

#define INSTRUCTION_BIT 64
#define INSTRUCTION_OP_BIT 8
#define INSTRUCTION_SRC1_MODE_BIT 4
#define INSTRUCTION_SRC1_BIT 24
#define INSTRUCTION_SRC2_MODE_BIT 4
#define INSTRUCTION_SRC2_BIT 24

#define INSTRUCTION_OP_HIGH (INSTRUCTION_BIT - 1)
#define INSTRUCTION_OP_LOW (INSTRUCTION_OP_HIGH - INSTRUCTION_OP_BIT + 1)

#define INSTRUCTION_SRC1_MODE_HIGH (INSTRUCTION_BIT - INSTRUCTION_OP_BIT - 1)
#define INSTRUCTION_SRC1_MODE_LOW (INSTRUCTION_SRC1_MODE_HIGH - INSTRUCTION_SRC1_MODE_BIT + 1)

#define INSTRUCTION_SRC1_HIGH (INSTRUCTION_BIT - INSTRUCTION_OP_BIT - INSTRUCTION_SRC1_MODE_BIT - 1)
#define INSTRUCTION_SRC1_LOW (INSTRUCTION_SRC1_HIGH - INSTRUCTION_SRC1_BIT + 1)

#define INSTRUCTION_SRC2_MODE_HIGH (INSTRUCTION_BIT - INSTRUCTION_OP_BIT - INSTRUCTION_SRC1_MODE_BIT - INSTRUCTION_SRC1_BIT - 1)
#define INSTRUCTION_SRC2_MODE_LOW (INSTRUCTION_SRC2_MODE_HIGH - INSTRUCTION_SRC2_MODE_BIT + 1)

#define INSTRUCTION_SRC2_HIGH (INSTRUCTION_BIT - INSTRUCTION_OP_BIT - INSTRUCTION_SRC1_MODE_BIT - INSTRUCTION_SRC1_BIT - INSTRUCTION_SRC2_MODE_BIT - 1)
#define INSTRUCTION_SRC2_LOW (INSTRUCTION_SRC2_HIGH - INSTRUCTION_SRC2_BIT + 1)

#define INSTRUCTION_OP_RANGE INSTRUCTION_OP_HIGH:INSTRUCTION_OP_LOW
#define INSTRUCTION_SRC1_MODE_RANGE INSTRUCTION_SRC1_MODE_HIGH:INSTRUCTION_SRC1_MODE_LOW
#define INSTRUCTION_SRC1_RANGE INSTRUCTION_SRC1_HIGH:INSTRUCTION_SRC1_LOW
#define INSTRUCTION_SRC2_MODE_RANGE INSTRUCTION_SRC2_MODE_HIGH:INSTRUCTION_SRC2_MODE_LOW
#define INSTRUCTION_SRC2_RANGE INSTRUCTION_SRC2_HIGH:INSTRUCTION_SRC2_LOW


#define INSTRUCTION_UNIT_SIZE (INSTRUCTION_BIT / MEMORY_UNIT_BIT)  // PC <- PC + INSTRUCTION_UNIT_SIZE
#define INSTRUCTION_MAX_SIZE 1024

#define JMP_FLAG_POSITION 1000

enum bit<4> INSTRUCTION_MODE_TYPE {
    REGISTER = 0x00,
    IMEDIATE = 0x01,
    MEMORY = 0x02,
    REGISTER_INDIRECT = 0x03,
    REGISTER_IMEDIATE = 0x04,
    REGISTER_REGISTER = 0x05,
    REGISTER_SCALE_REGISTER = 0x06,
    REGISTER_SCALE_IMEDIATE = 0x07,
    IMEDIATE_IMEDIATE = 0x08,
    IMEDIATE_SCALE_REGISTER = 0x09,
    IMEDIATE_SCALE_IMEDIATE = 0x0A,
}

enum bit<8> INSTRUCTION_OP_TYPE {
    ADD = 0x01,
    SUB = 0x02,
    MUL = 0x03,
    DIV = 0x04,
    SHL = 0x05,
    SHR = 0x06,
    MOV = 0x07,
    INC = 0x08,
    XOR = 0x09, 
    JL  = 0x0A,
    SET = 0x0B,
    EXTRACT = 0x0C,
    // LOAD = 0x07,
    // STORE = 0x08,
    // EXTRACT = 0x09,
    // MODIFY = 0x0A,
    // HASH = 0x0B,
    // CMP = 0x0C,
    // JMP = 0x0D,
    // JNZ = 0x0E,
}


enum bit<8> INSTRUCTION_COMMAND_TYPE {
    PUBLISH_PROGRAM = 0x00,
    EXECUTE_PROGRAM = 0x01,
}
/** INSTRUCTION END **/


#define GET_DATA(mode, index, data) { \
    if (mode == INSTRUCTION_MODE_TYPE.REGISTER) { \
        i_read_register(index, data); \
    } else if (mode == INSTRUCTION_MODE_TYPE.MEMORY) { \
        i_read_memory(index, data); \
    } else if (mode == INSTRUCTION_MODE_TYPE.REGISTER_INDIRECT) { \
        i_read_register(index, index); \
        i_read_memory(index, data); \
    } else if (mode == INSTRUCTION_MODE_TYPE.IMEDIATE) { \
        data = index; \
    } else if (mode == INSTRUCTION_MODE_TYPE.REGISTER_IMEDIATE) { \
        i_read_reg_imm(index, data);\
    } else if (mode == INSTRUCTION_MODE_TYPE.REGISTER_REGISTER) { \
        i_read_reg_reg(index, data);\
    } else if (mode == INSTRUCTION_MODE_TYPE.REGISTER_SCALE_REGISTER) { \
        i_read_reg_scale_reg(index, data);\
    }\
}

#define WRITE_DATA(mode, index, data) { \
    if (mode == INSTRUCTION_MODE_TYPE.REGISTER) { \
        i_write_register(index, data); \
    } else if (mode == INSTRUCTION_MODE_TYPE.MEMORY) { \
        i_write_memory(index, data); \
    } else if (mode == INSTRUCTION_MODE_TYPE.REGISTER_INDIRECT) { \
        i_read_register(index, index); \
        i_write_memory(index, data); \
    } \
}


#define EXECUTE_ONE_INSTRUCTION { \
    read_instruction(); \
    bit<INSTRUCTION_OP_BIT> op = meta.instruction[INSTRUCTION_OP_RANGE];\
    meta.src1_index = (bit<32>) meta.instruction[INSTRUCTION_SRC1_RANGE];\
    meta.src2_index = (bit<32>) meta.instruction[INSTRUCTION_SRC2_RANGE];\
    \
    meta.src1_mode = meta.instruction[INSTRUCTION_SRC1_MODE_RANGE];\
    meta.src2_mode = meta.instruction[INSTRUCTION_SRC2_MODE_RANGE];\
    \
    switch(op) {\
        INSTRUCTION_OP_TYPE.ADD: {\
            GET_DATA(meta.src1_mode, meta.src1_index, meta.src1_data);\
            GET_DATA(meta.src2_mode, meta.src2_index, meta.src2_data);\
            i_add(meta.src1_data, meta.src2_data);\
            WRITE_DATA(meta.src1_mode, meta.src1_index, meta.src1_data);\
        }\
        INSTRUCTION_OP_TYPE.MOV: {\
            GET_DATA(meta.src2_mode, meta.src2_index, meta.src2_data);\
            WRITE_DATA(meta.src1_mode, meta.src1_index, meta.src2_data);\
        }\
        INSTRUCTION_OP_TYPE.SHL: {\
            GET_DATA(meta.src1_mode, meta.src1_index, meta.src1_data);\
            GET_DATA(meta.src2_mode, meta.src2_index, meta.src2_data);\
            bit<8> src2_data = (bit<8>) meta.src2_data;\
            i_shl(meta.src1_data, src2_data);\
            WRITE_DATA(meta.src1_mode, meta.src1_index, meta.src1_data);\
        }\
        INSTRUCTION_OP_TYPE.INC: {\
            GET_DATA(meta.src1_mode, meta.src1_index, meta.src1_data);\
            WRITE_DATA(meta.src1_mode, meta.src1_index, meta.src1_data + 1);\
        }\
        INSTRUCTION_OP_TYPE.XOR: {\
            GET_DATA(meta.src1_mode, meta.src1_index, meta.src1_data);\
            GET_DATA(meta.src2_mode, meta.src2_index, meta.src2_data);\
            i_xor(meta.src1_data, meta.src2_data);\
            WRITE_DATA(meta.src1_mode, meta.src1_index, meta.src1_data);\
        }\
        INSTRUCTION_OP_TYPE.SET: {\
            GET_DATA(meta.src1_mode, meta.src1_index, meta.src1_data);\
            bit<32> jmp = JMP_FLAG_POSITION; \
            WRITE_DATA(INSTRUCTION_MODE_TYPE.MEMORY, jmp, meta.src1_data);\
        }\
        INSTRUCTION_OP_TYPE.JL: {\
            GET_DATA(meta.src1_mode, meta.src1_index, meta.src1_data);\
            GET_DATA(meta.src2_mode, meta.src2_index, meta.src2_data);\
            if (meta.src1_data < meta.src2_data) {\
                bit<32> jmp = JMP_FLAG_POSITION; \
                GET_DATA(INSTRUCTION_MODE_TYPE.MEMORY, jmp, meta.src1_data);\
                meta.pc = meta.src1_data;\
            }\
        }\
        INSTRUCTION_OP_TYPE.EXTRACT: {\
            i_read_header(meta.src2_index, meta.src2_data);\
            WRITE_DATA(meta.src1_mode, meta.src1_index, meta.src2_data);\
        }\
    }\
}



const bit<16> TYPE_IPV4 = 0x800;
const bit<16> PORT_INSTRUCTION = 203;  // UDP Port For Instruction Protocol



struct metadata {
    bit<1> should_execute_instruction;
    
    bit<INSTRUCTION_BIT> instruction;
    bit<32> pc;
    bit<32> flags;
    bit<32> r1;
    bit<32> r2;
    bit<32> r3;
    bit<32> r4;
    bit<32> r5;
    bit<32> r6;
    bit<32> r7;
    bit<32> r8;

    bit<INSTRUCTION_SRC1_MODE_BIT> src1_mode;
    bit<INSTRUCTION_SRC1_MODE_BIT> src2_mode;
    bit<32> src1_index;
    bit<32> src2_index;
    bit<32> src1_data;
    bit<32> src2_data;
}



register<bit<MEMORY_UNIT_BIT>>(MEMORY_MAX_SIZE) memory;



/*************************************************************************
*********************** P A R S E R  ***********************************
*************************************************************************/

parser MyParser(packet_in packet,
                out headers hdr,
                inout metadata meta,
                inout standard_metadata_t standard_metadata) {

    state start {
        transition parse_ethernet;
    }

    state parse_ethernet {
        packet.extract(hdr.ethernet);
        transition select(hdr.ethernet.etherType) {
            0x800: parse_ipv4;
            default: accept;
        }
    }

    state parse_ipv4 {
        packet.extract(hdr.ipv4);
        transition select(hdr.ipv4.protocol) {
            17: parse_udp;
            default: accept;
        }
    }
    
    state parse_udp {
        packet.extract(hdr.udp);
        transition select(hdr.udp.dstPort) {
            PORT_INSTRUCTION: parse_instruction;
            default: accept;
        }
    }

    state parse_instruction {
        packet.extract(hdr.instruction);
        transition select(hdr.instruction.command_type) {
            INSTRUCTION_COMMAND_TYPE.PUBLISH_PROGRAM: parse_data;
            INSTRUCTION_COMMAND_TYPE.EXECUTE_PROGRAM: parse_data;
            default: accept;
        }
    }

    state parse_data {
        packet.extract(hdr.data);
        transition accept;
    }

}


/*************************************************************************
************   C H E C K S U M    V E R I F I C A T I O N   *************
*************************************************************************/

control MyVerifyChecksum(inout headers hdr, inout metadata meta) {
    apply {  }
}


/*************************************************************************
**************  I N G R E S S   P R O C E S S I N G   *******************
*************************************************************************/

control MyIngress(inout headers hdr,
                  inout metadata meta,
                  inout standard_metadata_t standard_metadata) {


    action i_read_memory(in bit<32> index, inout bit<32> data) {
        memory.read(data, index);
    }

    action i_write_memory(in bit<32> index, in bit<32> data) {
        memory.write(index, data);
    }

    action i_read_memory64(inout bit<32> index, inout bit<64> data) {
        memory.read(data[63:32], index);
        memory.read(data[31:0], index+1);
    }

    action i_write_memory64(in bit<32> index, in bit<64> data) {
        memory.write(index, data[63:32]);
        memory.write(index+1, data[31:0]);
    }

    action i_pc_inc() {
        meta.pc = meta.pc + INSTRUCTION_UNIT_SIZE;
    }

    action i_read_register(in bit<32> index, inout bit<32> data) {
        if (index == 0) {
            data = meta.r1;
        } else if (index == 1) {
            data = meta.r2;
        } else if (index == 2) {
            data = meta.r3;
        } else if (index == 3) {
            data = meta.r4;
        }
    }

    action i_write_register(in bit<32> index, in bit<32> data) {
        if (index == 0) {
            meta.r1 = data;
        } else if (index == 1) {
            meta.r2 = data;
        } else if (index == 2) {
            meta.r3 = data;
        } else if (index == 3) {
            meta.r4 = data;
        }
    }

    action i_read_header(in bit<32> index, inout bit<32> data) {
        if (index == 0) {
            data = hdr.data.value00;
        } else if (index == 1) {
            data = hdr.data.value01;
        }
        // } else if (index == 2) {
        //     data = hdr.data.value02;
        // } else if (index == 3) {
        //     data = hdr.data.value03;
        // } else if (index == 4) {
        //     data = hdr.data.value04;
        // } else if (index == 5) {
        //     data = hdr.data.value05;
        // } else if (index == 6) {
        //     data = hdr.data.value06;
        // } else if (index == 7) {
        //     data = hdr.data.value07;
        // } else if (index == 8) {
        //     data = hdr.data.value08;
        // } else if (index == 9) {
        //     data = hdr.data.value09;
        // } else if (index == 10) {
        //     data = hdr.data.value10;
        // } else if (index == 11) {
        //     data = hdr.data.value11;
        // } else if (index == 12) {
        //     data = hdr.data.value12;
        // } else if (index == 13) {
        //     data = hdr.data.value13;
        // } else if (index == 14) {
        //     data = hdr.data.value14;
        // } else if (index == 15) {
        //     data = hdr.data.value15;
        // } else if (index == 16) {
        //     data = hdr.data.value16;
        // } else if (index == 17) {
        //     data = hdr.data.value17;
        // } else if (index == 18) {
        //     data = hdr.data.value18;
        // } else if (index == 19) {
        //     data = hdr.data.value19;
        // } else if (index == 20) {
        //     data = hdr.data.value20;
        // } else if (index == 21) {
        //     data = hdr.data.value21;
        // } else if (index == 22) {
        //     data = hdr.data.value22;
        // } else if (index == 23) {
        //     data = hdr.data.value23;
        // } else if (index == 24) {
        //     data = hdr.data.value24;
        // } else if (index == 25) {
        //     data = hdr.data.value25;
        // } else if (index == 26) {
        //     data = hdr.data.value26;
        // } else if (index == 27) {
        //     data = hdr.data.value27;
        // } else if (index == 28) {
        //     data = hdr.data.value28;
        // } else if (index == 29) {
        //     data = hdr.data.value29;
        // } else if (index == 30) {
        //     data = hdr.data.value30;
        // } else if (index == 31) {
        //     data = hdr.data.value31;
        // }

    }
    

    action i_read_reg_imm(in bit<32> index, inout bit<32> data) {
        bit<32> register_index = (bit<32>) index[23:20];
        i_read_register(register_index, data);
        bit<32> imediate = (bit<32>) index[19:0];
        data = data + imediate;
        i_read_memory(data, data);
    }

    action i_read_reg_reg(in bit<32> index, inout bit<32> data) {
        bit<32> register1_index = (bit<32>) index[23:20];
        i_read_register(register1_index, data);
        bit<32> register2_index = (bit<32>) index[19:16];
        bit<32> tmp = 0;
        i_read_register(register2_index, tmp);
        data = data + tmp;
        i_read_memory(data, data);
    }

    action i_read_reg_scale_reg(in bit<32> index, inout bit<32> data) {
        bit<32> register1_index = (bit<32>) index[23:20];
        i_read_register(register1_index, data);
        bit<32> register2_index = (bit<32>) index[19:16];
        bit<32> tmp = 0;
        i_read_register(register2_index, tmp);
        bit<32> scale = (bit<32>) index[15:12];
        data = data + scale * tmp;
        i_read_memory(data, data);
    }

    // action i_get_src1() {
    //     meta.src1_index = (bit<32>) meta.instruction[INSTRUCTION_SRC1_RANGE];
    //     memory.read(meta.src1_data, meta.src1_index);
    // }

    // action i_get_src2() {
    //     meta.src2_index = (bit<32>) meta.instruction[INSTRUCTION_SRC2_RANGE];
    //     memory.read(meta.src2_data, meta.src2_index);
    // }

    action i_add(inout bit<32> src1_data, inout bit<32> src2_data) {
        src1_data = src1_data + src2_data;
    }

    action i_shl(inout bit<32> src1_data, inout bit<8> src2_data) {
        src1_data = src1_data << src2_data;
    }

    action i_shr(inout bit<32> src1_data, inout bit<8> src2_data) {
        src1_data = src1_data >> src2_data;
    }

    action i_xor(inout bit<32> src1_data, inout bit<32> src2_data) {
        src1_data = src1_data ^ src2_data;
    }

    action read_instruction() {
        
        // memory.read(meta.instruction[63:32], meta.pc);
        // memory.read(meta.instruction[31:0], meta.pc+1);
        i_read_memory64(meta.pc, meta.instruction);
        meta.pc = meta.pc + INSTRUCTION_UNIT_SIZE;
        meta.should_execute_instruction = 1;
        // bit<8> op = meta.instruction[63:56];
        // switch(src1_index) {
        //     0x00: { i_add(); }
        //     default: {  }
        // }
        
        // if (op == 0x00) {
        //     i_add();
        // } else if (op == 0x01) {

        // }
    }

    action test_instruction() {
        bit<INSTRUCTION_BIT> instruction;
        bit<INSTRUCTION_OP_BIT> op = INSTRUCTION_OP_TYPE.ADD;
        bit<32> src1_index_ = 0x0010;
        bit<32> src2_index_ = 0x0011;
        bit<32> src1_data_ = 0x0010;
        bit<32> src2_data_ = 0x0011;
        instruction[INSTRUCTION_OP_RANGE] = (bit<INSTRUCTION_OP_BIT>) op;
        instruction[INSTRUCTION_SRC1_MODE_RANGE] = INSTRUCTION_MODE_TYPE.MEMORY;
        instruction[INSTRUCTION_SRC1_RANGE] = (bit<INSTRUCTION_SRC1_BIT>) src1_index_;
        instruction[INSTRUCTION_SRC2_MODE_RANGE] = INSTRUCTION_MODE_TYPE.MEMORY;
        instruction[INSTRUCTION_SRC2_RANGE] = (bit<INSTRUCTION_SRC2_BIT>) src2_index_;
        memory.write(meta.pc, instruction[63:32]);
        memory.write(meta.pc+1, instruction[31:0]);
        memory.write(src1_index_, src1_data_);
        memory.write(src2_index_, src2_data_);
        // instruction[]
    }

    action drop() {
        mark_to_drop(standard_metadata);
    }

    action ipv4_forward(macAddr_t dstAddr, egressSpec_t port) {

        // Set the src mac address as the previous dst, this is not correct right?
        hdr.ethernet.srcAddr = hdr.ethernet.dstAddr;

        // Set the destination mac address that we got from the match in the table.
        hdr.ethernet.dstAddr = dstAddr;

        // Set the output port that we also get from the table.
        standard_metadata.egress_spec = port;

        //decrease ttl by 1
        hdr.ipv4.ttl = hdr.ipv4.ttl -1;

    }

    table ipv4_lpm {
        key = {
            hdr.ipv4.dstAddr: lpm;
        }
        actions = {
            ipv4_forward;
            drop;
            NoAction;
        }
        size = 1024;
        default_action = NoAction();
    }

    // This included file contains the 'save_intructions' actions. These actions 
    // are generated automatically during build, according to the aggregation settings. 
    // They were extracted to their own file to make the generation process easier.
    #include "instruction.p4"

    apply {
        if (hdr.instruction.isValid()) {

            if (hdr.instruction.command_type == INSTRUCTION_COMMAND_TYPE.PUBLISH_PROGRAM) {
                save_intructions(hdr.instruction.start_pc);
            }

            else if (hdr.instruction.command_type == INSTRUCTION_COMMAND_TYPE.EXECUTE_PROGRAM) {

                meta.pc = hdr.instruction.start_pc;

                EXECUTE_ONE_INSTRUCTION;
                EXECUTE_ONE_INSTRUCTION;
                EXECUTE_ONE_INSTRUCTION;
                EXECUTE_ONE_INSTRUCTION;
                EXECUTE_ONE_INSTRUCTION;
                EXECUTE_ONE_INSTRUCTION;
                EXECUTE_ONE_INSTRUCTION;
                EXECUTE_ONE_INSTRUCTION;
                EXECUTE_ONE_INSTRUCTION;
                EXECUTE_ONE_INSTRUCTION;
                EXECUTE_ONE_INSTRUCTION;
                
                EXECUTE_ONE_INSTRUCTION;
            }
                    
        }

        // Only if IPV4 the rule is applied. Therefore other packets will not be forwarded.
        if (hdr.ipv4.isValid()){
            ipv4_lpm.apply();
        }
    }
}
/*************************************************************************
****************  E G R E S S   P R O C E S S I N G   *******************
*************************************************************************/

control MyEgress(inout headers hdr,
                 inout metadata meta,
                 inout standard_metadata_t standard_metadata) {

    apply {
        
    }
}

/*************************************************************************
*************   C H E C K S U M    C O M P U T A T I O N   **************
*************************************************************************/

control MyComputeChecksum(inout headers hdr, inout metadata meta) {
     apply {
	update_checksum(
	    hdr.ipv4.isValid(),
            { hdr.ipv4.version,
	      hdr.ipv4.ihl,
              hdr.ipv4.tos,
              hdr.ipv4.totalLen,
              hdr.ipv4.identification,
              hdr.ipv4.flags,
              hdr.ipv4.fragOffset,
              hdr.ipv4.ttl,
              hdr.ipv4.protocol,
              hdr.ipv4.srcAddr,
              hdr.ipv4.dstAddr },
            hdr.ipv4.hdrChecksum,
            HashAlgorithm.csum16);
    }
}


/*************************************************************************
***********************  D E P A R S E R  *******************************
*************************************************************************/

control MyDeparser(packet_out packet, in headers hdr) {
    apply {

        //parsed headers have to be added again into the packet.
        packet.emit(hdr.ethernet);
        packet.emit(hdr.ipv4);

    }
}

/*************************************************************************
***********************  S W I T C H  *******************************
*************************************************************************/

//switch architecture
V1Switch(
MyParser(),
MyVerifyChecksum(),
MyIngress(),
MyEgress(),
MyComputeChecksum(),
MyDeparser()
) main;
