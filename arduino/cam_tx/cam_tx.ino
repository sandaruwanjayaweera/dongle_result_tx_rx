#include <HardwareSerial.h>
#include "cam.hpp"

#define BT_RX_pin 16
#define BT_TX_pin 17

uint8_t pkt_no = 0;

pdu_src_station_id_t    pdu_src_station_id_r;
bc_ref_pos_lattitude_t  bc_ref_pos_lattitude_r;

void setup() {
  pinMode(22, OUTPUT);
  Serial.begin(9600);
  Serial2.begin(115200, SERIAL_8N1, BT_RX_pin, BT_TX_pin);
  Serial.println("Serial Txd is on pin: "+String(BT_TX_pin));
  Serial.println("Serial Rxd is on pin: "+String(BT_RX_pin)); 
}

void loop() {
  uint8_t recv_len 			  = 67;
  byte recv_msg[recv_len];
    while (Serial2.available()) {
      if(Serial2.read() == 255){
        if(Serial2.read() == 0){
          if(Serial2.read() == 255){
            int recv_size = Serial2.read();
            Serial2.readBytes(recv_msg, recv_len);
            pdu_src_station_id_r.bit_8[3] = recv_msg[2];
            pdu_src_station_id_r.bit_8[2] = recv_msg[3];
            pdu_src_station_id_r.bit_8[1] = recv_msg[4];
            pdu_src_station_id_r.bit_8[0] = recv_msg[5];
            bc_ref_pos_lattitude_r.bit_8[3] = recv_msg[7];
            bc_ref_pos_lattitude_r.bit_8[2] = recv_msg[8];
            bc_ref_pos_lattitude_r.bit_8[1] = recv_msg[9];
            bc_ref_pos_lattitude_r.bit_8[0] = recv_msg[10];
            Serial.println( " pdu_proto_version " + String(recv_msg[0]) + 
                            " pdu_message_id " + String(recv_msg[1]) + 
                            " pdu_src_station_id " + String(pdu_src_station_id_r.bit_32) + 
                            " bc_station_type " + String(recv_msg[6]) +
                            " bc_ref_pos_lattitude " + String(bc_ref_pos_lattitude_r.bit_32) +
                            " bc_ref_pos_longitude " + String(recv_msg[11]<<24 + recv_msg[12]<<16 + recv_msg[13]<<8 + recv_msg[14]) +
                            " bc_ref_pos_conf_ellipse_semi_major " + String(recv_msg[15]<<8 + recv_msg[16]) +
                            " bc_ref_pos_conf_ellipse_semi_minor " + String(recv_msg[17]<<8 + recv_msg[18]) + 
                            " bc_ref_pos_altitude_heading " + String(recv_msg[19]<<8 + recv_msg[20]) + 
                            " bc_ref_pos_altitude_val " + String(recv_msg[21]<<24 + recv_msg[22]<<16 + recv_msg[23]<<8 + recv_msg[24]) +
                            " bc_ref_pos_altitude_conf " + String(recv_msg[25]) +
                            " hf_heading_val " + String(recv_msg[26]<<8 + recv_msg[27]) +
                            " hf_heading_conf " + String(recv_msg[28]) +
                            " hf_vertical_heading " + String(recv_msg[29]<<8 + recv_msg[30]) +
                            " hf_speed_val " + String(recv_msg[31]<<8 + recv_msg[32]) +
                            " hf_speed_conf " + String(recv_msg[33]) +
                            " hf_drive_dir_heading_val " + String(recv_msg[34]<<8 + recv_msg[35]) +
                            " hf_drive_dir_heading_conf " + String(recv_msg[36]) +
                            " hf_drive_dir_vertical_heading " + String(recv_msg[37]<<8 + recv_msg[38]) +
                            " hf_v_len_val " + String(recv_msg[39]<<8 + recv_msg[40]) +
                            " hf_v_len_conf " + String(recv_msg[41]) +
                            " hf_v_width " + String(recv_msg[42]<<8 + recv_msg[43]) +
                            " hf_v_height " + String(recv_msg[44]<<8 + recv_msg[45]) +
                            " hf_long_acc_val " + String(recv_msg[46]<<8 + recv_msg[47]) +
                            " hf_long_acc_conf " + String(recv_msg[48]) +
                            " hf_curv_val " + String(recv_msg[49]<<8 + recv_msg[50]) +
                            " hf_curv_conf " + String(recv_msg[51]) +
                            " hf_curv_cal_mod " + String(recv_msg[52]) +
                            " hf_yaw_r_val " + String(recv_msg[53]<<8 + recv_msg[54]) +
                            " hf_yaw_r_conf " + String(recv_msg[55]) +
                            " hf_lat_acc_val " + String(recv_msg[56]<<8 + recv_msg[57]) +
                            " hf_lat_acc_conf " + String(recv_msg[58]) +
                            " hf_vertical_acc_val " + String(recv_msg[59]<<8 + recv_msg[60]) +
                            " hf_vertical_acc_conf " + String(recv_msg[61]) +
                            " uav_safetyarearadius " + String(recv_msg[62]<<8 + recv_msg[63]) +
                            " uav_pathhistory_len " + String(recv_msg[64])
            );
            break;
          }
        }
      }
    // Serial.print(char(Serial2.read()));
    // int recv_size = Serial2.readBytes(recv_msg, recv_len);
    // Serial.println("msg1 " + String(recv_msg[0]) + 
    // " msg2 " + String(recv_msg[1]) + 
    // " msg3 " + String(recv_msg[2]) + 
    // " msg4 " + String(recv_msg[3]) +
    // " msg5 " + String(recv_msg[4]) +
    // " msg6 " + String(recv_msg[5]) +
    // " msg7 " + String(recv_msg[6]) +
    // " msg8 " + String(recv_msg[7]) + 
    // " msg9 " + String(recv_msg[8]) + 
    // " msg10 " + String(recv_msg[9]) +
    // " msg11 " + String(recv_msg[10]) +
    // " msg12 " + String(recv_msg[11]) +
    // " msg13 " + String(recv_msg[12]) +
    // " msg14 " + String(recv_msg[13])
    // );
  }
  // digitalWrite(22, HIGH);
  // delay(100);
  // digitalWrite(22, LOW);
  // delay(100);

  pkt_no++;

  pdu_proto_version 	        = 1;
  pdu_message_id 		          = 2;
  pdu_src_station_id.bit_32   = 3;

  bc_station_type 					                  = 4;
  bc_ref_pos_lattitude.bit_32++;
  Serial.println("Sent lattitude: "+String(bc_ref_pos_lattitude.bit_32));

  bc_ref_pos_longitude.bit_32 				        = 6;
  bc_ref_pos_conf_ellipse_semi_major.bit_16 	= 7;
  bc_ref_pos_conf_ellipse_semi_minor.bit_16 	= 8;
  bc_ref_pos_altitude_heading.bit_16 		      = 9;
  bc_ref_pos_altitude_val.bit_32 			        = 10;
  bc_ref_pos_altitude_conf 			              = 11;

  hf_heading_val.bit_16 					        = 12;
  hf_heading_conf  					              = 13;
  hf_vertical_heading.bit_16 				      = 14;
  hf_speed_val.bit_16 						        = 15;
  hf_speed_conf 						              = 16;
  hf_drive_dir_heading_val.bit_16         = 17;
  hf_drive_dir_heading_conf 			        = 18;
  hf_drive_dir_vertical_heading.bit_16 		= 19;
  hf_v_len_val.bit_16 						        = 20;
  hf_v_len_conf 						              = 21;
  hf_v_width.bit_16 							        = 22;
  hf_v_height.bit_16 						          = 23;
  hf_long_acc_val.bit_16 					        = 24;
  hf_long_acc_conf 					              = 25;
  hf_curv_val.bit_16 						          = 26;
  hf_curv_conf 						                = 27;
  hf_curv_cal_mod 					              = 28;
  hf_yaw_r_val.bit_16 						        = 29;
  hf_yaw_r_conf 						              = 30;
  hf_lat_acc_val.bit_16 						      = 31;
  hf_lat_acc_conf 					              = 32;
  hf_vertical_acc_val.bit_16 				      = 33;
  hf_vertical_acc_conf 				            = 34;

  uav_safetyarearadius.bit_16 				    = 35;
  uav_pathhistory_len 				            = 36;


  uint8_t uart_len 			  = 71;

  byte msg[uart_len];

  msg[0] = 0xff;
  msg[1] = 0x00;
  msg[2] = 0xff;
  msg[3] = uart_len;      // length of data

	msg[4] = pdu_proto_version;
	msg[5] = pdu_message_id;
	msg[6] = pdu_src_station_id.bit_8[3];
	msg[7] = pdu_src_station_id.bit_8[2];
	msg[8] = pdu_src_station_id.bit_8[1];
	msg[9] = pdu_src_station_id.bit_8[0];

	msg[10]  = bc_station_type;
	msg[11] 	= bc_ref_pos_lattitude.bit_8[3];
	msg[12] 	= bc_ref_pos_lattitude.bit_8[2];
	msg[13] 	= bc_ref_pos_lattitude.bit_8[1];
	msg[14] = bc_ref_pos_lattitude.bit_8[0];
	msg[15] = bc_ref_pos_longitude.bit_8[3];
	msg[16] = bc_ref_pos_longitude.bit_8[2];
	msg[17] = bc_ref_pos_longitude.bit_8[1];
	msg[18] = bc_ref_pos_longitude.bit_8[0];
	msg[19] = bc_ref_pos_conf_ellipse_semi_major.bit_8[1];
	msg[20] = bc_ref_pos_conf_ellipse_semi_major.bit_8[0];
	msg[21] = bc_ref_pos_conf_ellipse_semi_minor.bit_8[1];
	msg[22] = bc_ref_pos_conf_ellipse_semi_minor.bit_8[0];
	msg[23] = bc_ref_pos_altitude_heading.bit_8[1];
	msg[24] = bc_ref_pos_altitude_heading.bit_8[0];
	msg[25] = bc_ref_pos_altitude_val.bit_8[3];
	msg[26] = bc_ref_pos_altitude_val.bit_8[2];
	msg[27] = bc_ref_pos_altitude_val.bit_8[1];
	msg[28] = bc_ref_pos_altitude_val.bit_8[0];
	msg[29] = bc_ref_pos_altitude_conf;

	msg[30] = hf_heading_val.bit_8[1];
	msg[31] = hf_heading_val.bit_8[0];
	msg[32] = hf_heading_conf;
	msg[33] = hf_vertical_heading.bit_8[1];
	msg[34] = hf_vertical_heading.bit_8[0];
	msg[35] = hf_speed_val.bit_8[1];
	msg[36] = hf_speed_val.bit_8[0];
	msg[37] = hf_speed_conf;
	msg[38] = hf_drive_dir_heading_val.bit_8[1];
	msg[39] = hf_drive_dir_heading_val.bit_8[0];
	msg[40] = hf_drive_dir_heading_conf;
	msg[41] = hf_drive_dir_vertical_heading.bit_8[1];
	msg[42] = hf_drive_dir_vertical_heading.bit_8[0];
	msg[43] = hf_v_len_val.bit_8[1];
	msg[44] = hf_v_len_val.bit_8[0];
	msg[45] = hf_v_len_conf;
	msg[46] = hf_v_width.bit_8[1];
	msg[47] = hf_v_width.bit_8[0];
	msg[48] = hf_v_height.bit_8[1];
	msg[49] = hf_v_height.bit_8[0];
	msg[50] = hf_long_acc_val.bit_8[1];
	msg[51] = hf_long_acc_val.bit_8[0];
	msg[52] = hf_long_acc_conf;
	msg[53] = hf_curv_val.bit_8[1];

	msg[54] = hf_curv_val.bit_8[0];
	msg[55] = hf_curv_conf;
	msg[56] = hf_curv_cal_mod;
	msg[57] = hf_yaw_r_val.bit_8[1];
	msg[58] = hf_yaw_r_val.bit_8[0];
	msg[59] = hf_yaw_r_conf;
	msg[60] = hf_lat_acc_val.bit_8[1];
	msg[61] = hf_lat_acc_val.bit_8[0];
	msg[62] = hf_lat_acc_conf;
	msg[63] = hf_vertical_acc_val.bit_8[1];
	msg[64] = hf_vertical_acc_val.bit_8[0];
	msg[65] = hf_vertical_acc_conf;

	msg[66] = uav_safetyarearadius.bit_8[1];
	msg[67] = uav_safetyarearadius.bit_8[0];
	msg[68] = uav_pathhistory_len;

  msg[69] = 0xff;
  msg[70] = 0xff;

  Serial2.write(msg, uart_len);
  delay(1000);
  // Serial.println(bytesSent);
  // Serial2.write(0x01);
  // Serial.print(char(Serial2.read()));
  // Serial.println("Loop done");
}
