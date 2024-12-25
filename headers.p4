
/*************************************************************************
*********************** H E A D E R S  ***********************************
*************************************************************************/

typedef bit<9>  egressSpec_t;
typedef bit<48> macAddr_t;
typedef bit<32> ip4Addr_t;

header ethernet_t {
    macAddr_t dstAddr;
    macAddr_t srcAddr;
    bit<16>   etherType;
}

header ipv4_t {
    bit<4>    version;
    bit<4>    ihl;
    bit<8>    tos;
    bit<16>   totalLen;
    bit<16>   identification;
    bit<3>    flags;
    bit<13>   fragOffset;
    bit<8>    ttl;
    bit<8>    protocol;
    bit<16>   hdrChecksum;
    ip4Addr_t srcAddr;
    ip4Addr_t dstAddr;
}


header udp_t {
    bit<16> srcPort;
    bit<16> dstPort;
    bit<16> length;
    bit<16> checksum;
}




header data_t {
    bit<32> value00;
    bit<32> value01;
    bit<32> value02;
    bit<32> value03;
    bit<32> value04;
    bit<32> value05;
    bit<32> value06;
    bit<32> value07;
    bit<32> value08;
    bit<32> value09;
    bit<32> value10;
    bit<32> value11;
    bit<32> value12;
    bit<32> value13;
    bit<32> value14;
    bit<32> value15;
    bit<32> value16;
    bit<32> value17;
    bit<32> value18;
    bit<32> value19;
    bit<32> value20;
    bit<32> value21;
    bit<32> value22;
    bit<32> value23;
    bit<32> value24;
    bit<32> value25;
    bit<32> value26;
    bit<32> value27;
    bit<32> value28;
    bit<32> value29;
    bit<32> value30;
    bit<32> value31;
}



header instruction_proto_t {
    bit<8> command_type;       
    bit<32> start_pc;           
    bit<16> instruction_count;  
}

struct headers {
    ethernet_t   ethernet;
    ipv4_t       ipv4;
    udp_t       udp;
    instruction_proto_t instruction;
    data_t      data;
}