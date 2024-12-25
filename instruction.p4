/**********************************************************************************
*********************** I N S T R U C T I O N  ************************************
**********************************************************************************/

action save_one_intruction(in bit<32> index, in bit<32> high_32, in bit<32> low_32) {
    i_write_memory(index, low_32);
    i_write_memory(index+1, high_32);
}

action save_intructions(in bit<32> ins_start_index) {
    save_one_intruction(ins_start_index, hdr.data.value01, hdr.data.value00);
    save_one_intruction(ins_start_index+2, hdr.data.value03, hdr.data.value02);
    save_one_intruction(ins_start_index+4, hdr.data.value05, hdr.data.value04);
    save_one_intruction(ins_start_index+6, hdr.data.value07, hdr.data.value06);
    save_one_intruction(ins_start_index+8, hdr.data.value09, hdr.data.value08);
    save_one_intruction(ins_start_index+10, hdr.data.value11, hdr.data.value10);
    save_one_intruction(ins_start_index+12, hdr.data.value13, hdr.data.value12);
    save_one_intruction(ins_start_index+14, hdr.data.value15, hdr.data.value14);
}











