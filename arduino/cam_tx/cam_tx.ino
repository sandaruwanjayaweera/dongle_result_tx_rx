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
  uint8_t uart_len 			  = 32;
  // byte recv_msg[uart_len];
  while (Serial2.available()) {
    // Serial2.readBytes(recv_msg, 65);
    // Serial.print(recv_msg[0]);
    Serial.print(char(Serial2.read()));
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

  byte msg[uart_len];

  msg[0] = 0xff;
  msg[1] = 0x00;
  msg[2] = 0xff;
  msg[3] = pkt_no;         // index of the dataset
  msg[4] = uart_len;      // length of data

	msg[5] = pdu_proto_version;
	msg[6] = pdu_message_id;
	msg[7] = pdu_src_station_id.bit_8[3];
	msg[8] = pdu_src_station_id.bit_8[2];
	msg[9] = pdu_src_station_id.bit_8[1];
	msg[10] = pdu_src_station_id.bit_8[0];

	msg[11]  = bc_station_type;
	msg[12] 	= bc_ref_pos_lattitude.bit_8[3];
	msg[13] 	= bc_ref_pos_lattitude.bit_8[2];
	msg[14] 	= bc_ref_pos_lattitude.bit_8[1];
	msg[15] = bc_ref_pos_lattitude.bit_8[0];
	msg[16] = bc_ref_pos_longitude.bit_8[3];
	msg[17] = bc_ref_pos_longitude.bit_8[2];
	msg[18] = bc_ref_pos_longitude.bit_8[1];
	msg[19] = bc_ref_pos_longitude.bit_8[0];
	msg[20] = bc_ref_pos_conf_ellipse_semi_major.bit_8[1];
	msg[21] = bc_ref_pos_conf_ellipse_semi_major.bit_8[0];
	msg[22] = bc_ref_pos_conf_ellipse_semi_minor.bit_8[1];
	msg[23] = bc_ref_pos_conf_ellipse_semi_minor.bit_8[0];
	msg[24] = bc_ref_pos_altitude_heading.bit_8[1];
	msg[25] = bc_ref_pos_altitude_heading.bit_8[0];
	msg[26] = bc_ref_pos_altitude_val.bit_8[3];
	msg[27] = bc_ref_pos_altitude_val.bit_8[2];
	msg[28] = bc_ref_pos_altitude_val.bit_8[1];
	msg[29] = bc_ref_pos_altitude_val.bit_8[0];

  msg[30] = 0xff;
  msg[31] = 0xff;

  Serial2.write(msg, uart_len);
  delay(30);

  msg[0] = 0xff;
  msg[1] = 0x00;
  msg[2] = 0xff;
  msg[3] = pkt_no;         // index of the dataset
  msg[4] = uart_len;      // length of data

	msg[5] = bc_ref_pos_altitude_conf;
	msg[6] = hf_heading_val.bit_8[1];
	msg[7] = hf_heading_val.bit_8[0];
	msg[8] = hf_heading_conf;
	msg[9] = hf_vertical_heading.bit_8[1];
	msg[10] = hf_vertical_heading.bit_8[0];
	msg[11] = hf_speed_val.bit_8[1];
	msg[12] = hf_speed_val.bit_8[0];
	msg[13] = hf_speed_conf;
	msg[14] = hf_drive_dir_heading_val.bit_8[1];
	msg[15] = hf_drive_dir_heading_val.bit_8[0];
	msg[16] = hf_drive_dir_heading_conf;
	msg[17] = hf_drive_dir_vertical_heading.bit_8[1];
	msg[18] = hf_drive_dir_vertical_heading.bit_8[0];
	msg[19] = hf_v_len_val.bit_8[1];
	msg[20] = hf_v_len_val.bit_8[0];
	msg[21] = hf_v_len_conf;
	msg[22] = hf_v_width.bit_8[1];
	msg[23] = hf_v_width.bit_8[0];
	msg[24] = hf_v_height.bit_8[1];
	msg[25] = hf_v_height.bit_8[0];
	msg[26] = hf_long_acc_val.bit_8[1];
	msg[27] = hf_long_acc_val.bit_8[0];
	msg[28] = hf_long_acc_conf;
	msg[29] = hf_curv_val.bit_8[1];

  msg[30] = 0xff;
  msg[31] = 0xff;

  Serial2.write(msg, uart_len);
  delay(30);

  msg[0] = 0xff;
  msg[1] = 0x00;
  msg[2] = 0xff;
  msg[3] = pkt_no;         // index of the dataset
  msg[4] = uart_len;      // length of data

	msg[5] = hf_curv_val.bit_8[0];
	msg[6] = hf_curv_conf;
	msg[7] = hf_curv_cal_mod;
	msg[8] = hf_yaw_r_val.bit_8[1];
	msg[9] = hf_yaw_r_val.bit_8[0];
	msg[10] = hf_yaw_r_conf;
	msg[11] = hf_lat_acc_val.bit_8[1];
	msg[12] = hf_lat_acc_val.bit_8[0];
	msg[13] = hf_lat_acc_conf;
	msg[14] = hf_vertical_acc_val.bit_8[1];
	msg[15] = hf_vertical_acc_val.bit_8[0];
	msg[16] = hf_vertical_acc_conf;

	msg[17] = uav_safetyarearadius.bit_8[1];
	msg[18] = uav_safetyarearadius.bit_8[0];
	msg[19] = uav_pathhistory_len;

  msg[20] = 0;
  msg[21] = 0;
  msg[22] = 0;
  msg[23] = 0;
  msg[24] = 0;
  msg[25] = 0;
  msg[26] = 0;
  msg[27] = 0;
  msg[28] = 0;
  msg[29] = 0;

  msg[30] = 0xff;
  msg[31] = 0xff;

  Serial2.write(msg, uart_len);
  delay(30);
  // Serial.println(bytesSent);
  // Serial2.write(0x01);
  // Serial.print(char(Serial2.read()));
  // Serial.println("Loop done");
}
