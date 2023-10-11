#include <HardwareSerial.h>
#include "cam.hpp"

#define BT_RX_pin 16
#define BT_TX_pin 17

void setup() {
  pinMode(22, OUTPUT);
  Serial.begin(9600);
  Serial2.begin(115200, SERIAL_8N1, BT_RX_pin, BT_TX_pin);
  Serial.println("Serial Txd is on pin: "+String(BT_TX_pin));
  Serial.println("Serial Rxd is on pin: "+String(BT_RX_pin));  
}

void loop() {
    while (Serial2.available()) {
    Serial.print(char(Serial2.read()));
  }
  digitalWrite(22, HIGH);
  delay(100);
  digitalWrite(22, LOW);
  delay(100);


  uint8_t uart_len 			  = 65;

	pdu_proto_version 	        = 1;
	pdu_message_id 		          = 2;
	pdu_src_station_id.bit_32   = 3;

	bc_station_type 					                  = 4;
	bc_ref_pos_lattitude.bit_32 				        = 5;
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

  int msg_len = uart_len + 7;
  byte msg[msg_len];
  byte msg[uart_len];

  // msg[0] = 0xff;
  // msg[1] = 0x00;
  // msg[2] = 0xff;
  // msg[3] = uart_len;  // length of data

	msg[0] = pdu_proto_version;
	msg[1] = pdu_message_id;
	msg[2] = pdu_src_station_id.bit_8[3];
	msg[3] = pdu_src_station_id.bit_8[2];
	msg[4] = pdu_src_station_id.bit_8[1];
	msg[5] = pdu_src_station_id.bit_8[0];

	msg[6]  = bc_station_type;
	msg[7] 	= bc_ref_pos_lattitude.bit_8[3];
	msg[8] 	= bc_ref_pos_lattitude.bit_8[2];
	msg[9] 	= bc_ref_pos_lattitude.bit_8[1];
	msg[10] = bc_ref_pos_lattitude.bit_8[0];
	msg[11] = bc_ref_pos_longitude.bit_8[3];
	msg[12] = bc_ref_pos_longitude.bit_8[2];
	msg[13] = bc_ref_pos_longitude.bit_8[1];
	msg[14] = bc_ref_pos_longitude.bit_8[0];
	msg[15] = bc_ref_pos_conf_ellipse_semi_major.bit_8[1];
	msg[16] = bc_ref_pos_conf_ellipse_semi_major.bit_8[0];
	msg[17] = bc_ref_pos_conf_ellipse_semi_minor.bit_8[1];
	msg[18] = bc_ref_pos_conf_ellipse_semi_minor.bit_8[0];
	msg[19] = bc_ref_pos_altitude_heading.bit_8[1];
	msg[20] = bc_ref_pos_altitude_heading.bit_8[0];
	msg[21] = bc_ref_pos_altitude_val.bit_8[3];
	msg[22] = bc_ref_pos_altitude_val.bit_8[2];
	msg[23] = bc_ref_pos_altitude_val.bit_8[1];
	msg[24] = bc_ref_pos_altitude_val.bit_8[0];
	msg[25] = bc_ref_pos_altitude_conf;

	msg[26] = hf_heading_val.bit_8[1];
	msg[27] = hf_heading_val.bit_8[0];
	msg[28] = hf_heading_conf;
	msg[29] = hf_vertical_heading.bit_8[1];
	msg[30] = hf_vertical_heading.bit_8[0];
	msg[31] = hf_speed_val.bit_8[1];
	msg[32] = hf_speed_val.bit_8[0];
	msg[33] = hf_speed_conf;
	msg[34] = hf_drive_dir_heading_val.bit_8[1];
	msg[35] = hf_drive_dir_heading_val.bit_8[0];
	msg[36] = hf_drive_dir_heading_conf;
	msg[37] = hf_drive_dir_vertical_heading.bit_8[1];
	msg[38] = hf_drive_dir_vertical_heading.bit_8[0];
	msg[39] = hf_v_len_val.bit_8[1];
	msg[40] = hf_v_len_val.bit_8[0];
	msg[41] = hf_v_len_conf;
	msg[42] = hf_v_width.bit_8[1];
	msg[43] = hf_v_width.bit_8[0];
	msg[44] = hf_v_height.bit_8[1];
	msg[45] = hf_v_height.bit_8[0];
	msg[46] = hf_long_acc_val.bit_8[1];
	msg[47] = hf_long_acc_val.bit_8[0];
	msg[48] = hf_long_acc_conf;
	msg[49] = hf_curv_val.bit_8[1];
	msg[50] = hf_curv_val.bit_8[0];
	msg[51] = hf_curv_conf;
	msg[52] = hf_curv_cal_mod;
	msg[53] = hf_yaw_r_val.bit_8[1];
	msg[54] = hf_yaw_r_val.bit_8[0];
	msg[55] = hf_yaw_r_conf;
	msg[56] = hf_lat_acc_val.bit_8[1];
	msg[57] = hf_lat_acc_val.bit_8[0];
	msg[58] = hf_lat_acc_conf;
	msg[59] = hf_vertical_acc_val.bit_8[1];
	msg[60] = hf_vertical_acc_val.bit_8[0];
	msg[61] = hf_vertical_acc_conf;

	msg[62] = uav_safetyarearadius.bit_8[1];
	msg[63] = uav_safetyarearadius.bit_8[0];
	msg[64] = uav_pathhistory_len;

  // msg[69] = 0xff;
  // msg[70] = 0xff;
  // msg[71] = 0xff;

  // int bytesSent = Serial2.write(msg, msg_len);
  int bytesSent = Serial2.write(msg, uart_len);
  delay(100);
  // Serial.println(bytesSent);
  // Serial2.write(0x01);
  // Serial.print(char(Serial2.read()));
  // Serial.println("Loop done");
}
