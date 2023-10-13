#include <HardwareSerial.h>
#include "cam.hpp"

#define BT_RX_pin 16
#define BT_TX_pin 17

uint8_t pkt_no = 0;

void setup() {
  pinMode(22, OUTPUT);
  Serial.begin(9600);
  Serial2.begin(115200, SERIAL_8N1, BT_RX_pin, BT_TX_pin);
  Serial.println("Serial Txd is on pin: "+String(BT_TX_pin));
  Serial.println("Serial Rxd is on pin: "+String(BT_RX_pin)); 
}

void loop() {
  uint8_t recv_len 			  = 65;
  byte recv_msg[recv_len];
    while (Serial2.available()) {
    // Serial.print(char(Serial2.read()));
    int recv_size = Serial2.readBytes(recv_msg, recv_len);
    Serial.println(recv_msg[0]);
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
