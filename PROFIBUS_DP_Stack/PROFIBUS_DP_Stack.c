/*
 * PROFIBUS_DP_Slave.c
 */

void PROFIBUS_begin()
{   
	
	#ifdef __HARDWARE_TARGET_ARDUINO__
		pinMode(pin_RE, OUTPUT);
		pinMode(pin_DE, OUTPUT);

		digitalWrite(pin_RE, HIGH);
		digitalWrite(pin_DE, LOW);

		Timer1.initialize(11*TBIT*255);
		Timer1.stop();

	#endif

		RX_size = 0;
		TX_size = 0;

		// Station Type
		station = PROFIBUS_SLAVE;
		ADDR = 126;

	#ifdef __HARDWARE_TARGET_ARDUINO__
		ident_H = 0x00;
		ident_L = 0x04;
	#endif
	#ifdef __HARDWARE_TARGET_NXT__
		ident_H = 0x00;
		ident_L = 0x05;
	#endif
	
	// Slave settings
	master_address = 0xFF; // No master when initialised
	DIAG_1.byte = 0x00;
	DIAG_2.byte = 0x04;
	DIAG_3.byte = 0x00;
	DIAG_station_non_existent = 0;
	DIAG_station_not_ready = 1;
	DIAG_configuration_fault = 0;
	DIAG_extended_diagnostic = 0;
	DIAG_not_supported = 0;
	DIAG_invalid_slave_response = 0;
	DIAG_parameter_fault = 0;
	DIAG_master_lock = 0;
	DIAG_parameter_request = 1;
	DIAG_status_diagnostics = 0;
	DIAG_watchdog_on = 0;
	DIAG_freeze_mode = 0;
	DIAG_sync_mode = 0;
	DIAG_deactivated = 0;
	DIAG_diagnostic_overflow = 0;
	
	Data_Exch_Packet = 0;
	
	BUS_threshold = 10000;
	
	RW_State = PROFIBUS_READSTART;
	S_State = PROFIBUS_SS_Power_ON;
	MS_State = PROFIBUS_MS_CLEAR;
	
}

void PROFIBUS_setConfig(const uint8_t t_Inputs, const uint8_t t_Outputs, const uint8_t t_Parameters){
	number_Of_Inputs = t_Inputs;
	number_Of_Outputs = t_Outputs;
	number_Of_Parameters = DEVICE_PARAMETERS + t_Parameters;
	STORED_OUT = (uint8_t *) malloc(t_Outputs);
}

void PROFIBUS_start()
{
	#ifdef __HARDWARE_TARGET_ARDUINO__
		Serial_PB.begin(BAUDRATE,SERIAL_8E1);
	#endif
	#ifdef __HARDWARE_TARGET_NXT__
		hs_init();
		hs_enable(BAUDRATE);
	#endif
	#ifdef PC_DEBUGGING
		Serial_Debug.begin(BAUDRATE);
	#endif
}


void PROFIBUS_run()
{
	PROFIBUS_S_State();
	switch(RW_State)
	{
		case PROFIBUS_IDLE:		
			break;
		case PROFIBUS_READSTART:
			PROFIBUS_readstart();
			break;
		case PROFIBUS_READ:
			PROFIBUS_read();
			break;
		case PROFIBUS_WRITESTART:
			PROFIBUS_writestart();
			break;
		case PROFIBUS_WRITE:
			PROFIBUS_write();
			break;
		case PROFIBUS_WRITESTOP:
			PROFIBUS_writestop();
			break;
		default:
			RW_State = PROFIBUS_IDLE;
			break;
	}
}

void PROFIBUS_S_State(){
	if(BUS_active){
		#ifdef __HARDWARE_TARGET_ARDUINO__
			BUS_curr_time = millis();
			if(BUS_curr_time - BUS_last_time > BUS_threshold){
		#endif
		#ifdef __HARDWARE_TARGET_NXT__
			BUS_curr_time++;
			if(BUS_curr_time > BUS_threshold){
		#endif		
				Data_Exch_Packet = 0;
				DIAG_station_not_ready = 1;
				DIAG_parameter_request = 1;
				DIAG_configuration_fault = 1;
				FailSafe();
				S_State = PROFIBUS_SS_Power_ON;
				BUS_active = 0;
		}
	}
	
	switch(S_State){
		case PROFIBUS_SS_Data_XCHG:
			if(DIAG_watchdog_on){
				#ifdef __HARDWARE_TARGET_ARDUINO__
					WD_curr_time = millis();
					if(WD_curr_time - WD_last_time > WD_threshold){
				#endif
				#ifdef __HARDWARE_TARGET_NXT__
					WD_curr_time++;
					if(WD_curr_time > WD_threshold){
				#endif
						Data_Exch_Packet = 0;
						DIAG_station_not_ready = 1;
						DIAG_parameter_request = 1;
						FailSafe();					
						S_State = PROFIBUS_SS_Wait_PRM;
						break;
				}
			}
			if(DIAG_parameter_fault){
				S_State = PROFIBUS_SS_Wait_CFG;
			}else if(DIAG_configuration_fault){
				S_State = PROFIBUS_SS_Wait_PRM;
			}else if(DIAG_station_not_ready || MS_State == PROFIBUS_MS_CLEAR){
				for(i=0;i<number_Of_Outputs;i++){
					OUT[i] = 0x00;
					STORED_OUT[i] = 0x00;
				}
				S_State = PROFIBUS_SS_Pre_Data_XCHG;
			}
			break;
		case PROFIBUS_SS_Pre_Data_XCHG:
			if(MS_State == PROFIBUS_MS_OPERATE && !DIAG_station_not_ready && Data_Exch_Packet){
				S_State = PROFIBUS_SS_Data_XCHG;
			}
			break;
		case PROFIBUS_SS_Power_ON:
			if(ADDR >= 0 && ADDR <= 125 && BUS_active){
				S_State = PROFIBUS_SS_Wait_PRM;
			}
			break;
		case PROFIBUS_SS_Wait_PRM:
			if(DIAG_parameter_fault == 0 && DIAG_parameter_request == 0){ // CHECK IF PARAM OK
				S_State = PROFIBUS_SS_Wait_CFG;
			}
			break;
		case PROFIBUS_SS_Wait_CFG:
			if(!DIAG_configuration_fault){
				S_State = PROFIBUS_SS_Pre_Data_XCHG;
			}
			break;
		default:
			S_State = PROFIBUS_SS_Power_ON;
			break;
	}

}

void PROFIBUS_stop()
{
	#ifdef __HARDWARE_TARGET_ARDUINO__
		Serial_PB.end();
	#endif
}

void PROFIBUS_writestart()
{
	#ifdef __HARDWARE_TARGET_ARDUINO__
		if(Serial_PB.available()){
			Serial_PB.read();
		}else{
			digitalWrite(pin_RE,HIGH);
			delayMicroseconds(1*TBIT);
			digitalWrite(pin_DE, HIGH);
			delayMicroseconds(5*TBIT);
			RW_State = PROFIBUS_WRITE;
			PROFIBUS_write();
		}
	#endif
	#ifdef __HARDWARE_TARGET_NXT__
		RW_State = PROFIBUS_WRITE;
		PROFIBUS_write();
	#endif
}

void PROFIBUS_write()
{
	#ifdef __HARDWARE_TARGET_ARDUINO__
		Serial_PB.write(TX_buf,TX_size);
		RW_State = PROFIBUS_IDLE;
		Timer1.start();
		Timer1.attachInterrupt(PROFIBUS_writestop,TX_size*11*TBIT);
	#endif
	#ifdef __HARDWARE_TARGET_NXT__
		hs_write(TX_buf,0,TX_size);
		RW_State = PROFIBUS_WRITESTOP;
	#endif
}
void PROFIBUS_writestop()
{	
	#ifdef __HARDWARE_TARGET_ARDUINO__
		Timer1.stop();
		Timer1.detachInterrupt();
		Serial_PB.flush();
		digitalWrite(pin_DE, LOW);
		RW_State = PROFIBUS_READSTART;
		PROFIBUS_readstart();
	#endif
	#ifdef __HARDWARE_TARGET_NXT__
		RW_State = PROFIBUS_READSTART;
		PROFIBUS_readstart();
	#endif
}

void PROFIBUS_readstart()
{
	#ifdef __HARDWARE_TARGET_ARDUINO__
		digitalWrite(pin_RE, LOW);
	#endif
	RX_size = 0;
	RX_pointer = 0;
	RW_State = PROFIBUS_READ;
}

void PROFIBUS_read()
{
	
	RX_prev_size = RX_size;
	#ifdef __HARDWARE_TARGET_ARDUINO__
		while(Serial_PB.available()){
			RX_buf[RX_size+RX_pointer]=Serial_PB.read();
			RX_size++;
		}
	#endif
	
	#ifdef __HARDWARE_TARGET_NXT__
		RX_size += hs_read(RX_buf,RX_size+RX_pointer,1024);
	#endif
	
	if(RX_prev_size != RX_size){
		BUS_active = 1;
		#ifdef __HARDWARE_TARGET_ARDUINO__
			BUS_last_time = millis();
		#endif
		#ifdef __HARDWARE_TARGET_NXT__
			BUS_curr_time = 0;
		#endif
	}
	while(RX_size>0 && RW_State == PROFIBUS_READ){
		if(RX_pointer>=768){
			// Removing Telegrams from Buffer
			for(i=0;i<RX_size;i++){
				RX_buf[i] = RX_buf[i+RX_pointer];
			}
			RX_pointer = 0;
		}
		process_packet(&RX_buf[RX_pointer]);
		if(RW_State == PROFIBUS_READ_NEXT_BYTE){
			RX_size--;
			RX_pointer++;
			RW_State = PROFIBUS_READ;
		}
	}	
	if(RW_State == PROFIBUS_WRITESTART){
		PROFIBUS_writestart();
		return;
	}
	if(RW_State == PROFIBUS_READ_MORE){
		RW_State = PROFIBUS_READ;	
	}
	if(RX_size == 0){
		RX_pointer = 0;
	}
}

void createSD1(const uint8_t DA,const uint8_t SA,const uint8_t FC)
{
	TX_buf[0] = PROFIBUS_SD1;
	TX_buf[1] = DA;
	TX_buf[2] = SA;
	TX_buf[3] = FC;
	TX_buf[4] = CRC(&TX_buf[1],3);
	TX_buf[5] = 0x16;
	TX_size = 6;
}

void createSD2(const uint8_t DA,const uint8_t SA,const uint8_t FC,const size_t size)
{
	TX_buf[0] = PROFIBUS_SD2;
	TX_buf[1] = 3+size;
	TX_buf[2] = 3+size;
	TX_buf[3] = PROFIBUS_SD2;
	TX_buf[4] = DA;
	TX_buf[5] = SA;
	TX_buf[6] = FC;
	// CONTENT FILLED BY FUNCTION CALLER
	TX_buf[7+size] = CRC(&TX_buf[4],3+size);
	TX_buf[8+size] = 0x16;
	TX_size = 9+size;
}


void createSD4(const uint8_t DA,const uint8_t SA)
{
	TX_buf[0] = PROFIBUS_SD4;
	TX_buf[1] = DA;
	TX_buf[2] = SA;
	TX_size = 3;
}

void createSC()
{
	TX_buf[0] = PROFIBUS_SC;
	TX_size = 1;
}


void process_packet(const uint8_t *buf)
{
	switch(buf[0]){
		case PROFIBUS_SD1:
			if(RX_size >= 6){
				if(buf[5]!=0x16 || buf[4]!=CRC(&buf[1],3)){
					// BAD PACKET
					RW_State = PROFIBUS_READ_NEXT_BYTE;
				}else{
					if((buf[1]&0x7F)!=ADDR){
						// Removing SD1 Telegram from Buffer
						RW_State = PROFIBUS_READ_NEXT_PACKET;
					}else{
						// GOOD PACKET
						RW_State = PROFIBUS_READ_NEXT_PACKET;
						switch(buf[3]&0x0F){		
							case 0x00: // TE: Time Event (Clock synchronisation)
								break;
							case 0x03: // SDA_Low: Send Data Acknowledged (Low priority)
								break;
							case 0x04: // SDN_Low: Send Data Not acknowledged (Low priority)
								break;
							case 0x05: // SDA_High: Send Data Acknowledged (High priority)
								break;
							case 0x06: // SDN_High: Send Data Not acknowledged (High priority)
								break;
							case 0x07: // MSRD: Send Request Data with Multicast Reply
								break;
							case 0x09: // Request FDL Status
								createSD1(buf[2],ADDR,station);
								RW_State = PROFIBUS_WRITESTART;
								break;
							case 0x0C: // SRD_Low: Send and Request Data
								break;
							case 0x0D: // SRD_High: Send and Request Data
								createSC();
								RW_State = PROFIBUS_WRITESTART;
								break;
							case 0x0E: // Request Ident with reply
								break;
							case 0x0F: // Request LSAP Status with reply
								createSC();
								RW_State = PROFIBUS_WRITESTART;
								break;
							case 0x80: // CV: Clock Value (Clock synchronisation)
								break;
						}
					}
				}
				if(RW_State == PROFIBUS_READ_NEXT_PACKET){
					RX_pointer = RX_pointer+6;
					RX_size = RX_size-6;
					RW_State = PROFIBUS_READ;
				}
			}else{
				RW_State = PROFIBUS_READ_MORE;
			}
			break;
	   case PROFIBUS_SD2:
			if(RX_size >= 4){
				if(buf[1]!=buf[2] || buf[3]!=PROFIBUS_SD2){
					// BAD PACKET
					RW_State = PROFIBUS_READ_NEXT_BYTE;
				}else{
					if(RX_size>=buf[1]+6){
						if(buf[buf[1]+5]!=0x16 || buf[buf[1]+4]!=CRC(&buf[4],buf[1])){
							// BAD PACKET
							RW_State = PROFIBUS_READ_NEXT_BYTE;
						}else{
							if((buf[4]&0x7F)!=ADDR && (buf[4]&0x7F)!=127){
								// Removing SD2 Telegram from Buffer
								RW_State = PROFIBUS_READ_NEXT_PACKET;
							}else{
								// GOOD PACKET
								RW_State = PROFIBUS_READ_NEXT_PACKET;
								SA = buf[5]&0x7F;
								DA = buf[4]&0x7F;
								FC = buf[6];
								if(buf[4]>>7 || buf[5]>>7){
									// SAP
									process_SAP(&buf[7],buf[1]-3);
								}else{
									// NO SAP
									if(S_State >= PROFIBUS_SS_Pre_Data_XCHG){
										// DATA EXCHANGE
										Data_Exch_Packet = 1;
										#ifdef __HARDWARE_TARGET_ARDUINO__
											WD_last_time = millis();
										#endif
										#ifdef __HARDWARE_TARGET_NXT__
											WD_curr_time=0;
										#endif
										for(i=0;i<number_Of_Inputs;i++){
											TX_buf[7+i] = IN[i];
										}
										if(buf[1]==3){
											// No Data, Clear mode?
											FailSafe();
											createSD2(SA,DA,station|0x8,number_Of_Inputs);
										}else if(buf[1]==3+number_Of_Outputs){
											if(S_State == PROFIBUS_SS_Data_XCHG){
												if(DIAG_sync_mode == 0){
													for(i=0;i<number_Of_Outputs;i++){
														OUT[i] = buf[7+i];
														STORED_OUT[i] = buf[7+i];
													}
												}else{
													for(i=0;i<number_Of_Outputs;i++){
														STORED_OUT[i] = buf[7+i];
													}
												}
												createSD2(SA,DA,station|0x8,number_Of_Inputs);
											}else{
												FailSafe();
												if(DIAG_station_not_ready){
													createSD2(SA,DA,station|0xA,number_Of_Inputs);
												}else{
													createSD2(SA,DA,station|0x8,number_Of_Inputs);
												}
											}
										}else{
											// Not supported, answer with SR
											createSD1(SA,DA,station|0x3);
										}
									}else{
										// NO DATA EXCHANGE
										createSD1(SA,DA,station|0x3);
									}
									RW_State = PROFIBUS_WRITESTART;
								}
							}
						}
					}else{
						RW_State = PROFIBUS_READ_MORE;
					}
				}
				if(RW_State == PROFIBUS_READ_NEXT_PACKET){
					RX_pointer = RX_pointer+(buf[1]+6);
					RX_size = RX_size-(buf[1]+6);
					RW_State = PROFIBUS_READ;
				}
			}else{
				RW_State = PROFIBUS_READ_MORE;
			}
			break;
		case PROFIBUS_SD3:
			if(RX_size >= 14){
				if(buf[13]!=0x16 || buf[12]!=CRC(&buf[1],11)){
					// BAD PACKET
					RW_State = PROFIBUS_READ_NEXT_BYTE;
				}else{
					// Removing SD3 Telegram from Buffer
					RX_pointer=RX_pointer+14;
					RX_size = RX_size-14;
					RW_State = PROFIBUS_READ;
				}
			}else{
				RW_State = PROFIBUS_READ_MORE;
			}
			break;
		/*case PROFIBUS_SD4:
			RW_State = PROFIBUS_READ_NEXT_BYTE;
			break;*/
		/*case PROFIBUS_SC:
			RW_State = PROFIBUS_READ_NEXT_BYTE;
			break;*/
		default:
			RW_State = PROFIBUS_READ_NEXT_BYTE;
			break;
	}
}

void process_SAP(const uint8_t *buf, const size_t size)
{
	switch(buf[1])
	{
		case SSAP_DP_MS0:
			switch(buf[0])
			{
				case DSAP_SET_SLAVE_ADDRESS:
					ADDR = buf[2];
					if(station!=PROFIBUS_SLAVE){
						station = PROFIBUS_MASTER_NOT_READY;
					}
					createSC();
					RW_State = PROFIBUS_WRITESTART;
					break;
				case DSAP_READ_INPUT:
					TX_buf[7]=buf[1];
					TX_buf[8]=DSAP_READ_INPUT;
					for(i=0;i<number_Of_Inputs;i++){
						TX_buf[9+i] = IN[i];
					}
					createSD2(SA|0x80,ADDR|0x80,station|0x08,number_Of_Inputs+2);
					RW_State = PROFIBUS_WRITESTART;
					break;
				case DSAP_READ_OUTPUT:
					TX_buf[7]=buf[1];
					TX_buf[8]=DSAP_READ_INPUT;
					for(i=0;i<number_Of_Outputs;i++){
						TX_buf[9+i] = OUT[i];
					}
					createSD2(SA|0x80,ADDR|0x80,station|0x08,number_Of_Outputs+2);
					RW_State = PROFIBUS_WRITESTART;
					break;
				case DSAP_GLOBAL_CONTROL:
					if(buf[2] == 0x20) // SYNC && (buf[3]&group_allocation)))
					{
						for(i=0;i<number_Of_Outputs;i++){
							OUT[i] = STORED_OUT[i];
						}
						DIAG_sync_mode = 1;
					}
					if(buf[2] == 0x10) // UNSYNC && (buf[3]&group_allocation)))
					{
						for(i=0;i<number_Of_Outputs;i++){
							OUT[i] = STORED_OUT[i];
						}
						DIAG_sync_mode = 0;
					}
					if(buf[2] == 0x02) // CLEAR MODE
					{
						MS_State = PROFIBUS_MS_CLEAR;
					}
					if(buf[2] == 0x00) // OPERATE MODE
					{
						MS_State = PROFIBUS_MS_OPERATE;
					}
					break;
				case DSAP_GET_CONFIG:
					TX_buf[7]=buf[1];
					TX_buf[8]=DSAP_GET_CONFIG;
					for(i=0;i<number_Of_Inputs;i++){
						TX_buf[9+i] = 0x10;
					}
					for(i=0;i<number_Of_Outputs;i++){
						TX_buf[9+number_Of_Inputs+i] = 0x20;
					}
					createSD2(SA|0x80,ADDR|0x80,station|0x08,number_Of_Inputs+number_Of_Outputs+2);
					RW_State = PROFIBUS_WRITESTART;
					break;
				case DSAP_SLAVE_DIAGNOSIS:
					TX_buf[7]=buf[1];
					TX_buf[8]=DSAP_SLAVE_DIAGNOSIS;
					TX_buf[9]=DIAG_1.byte;
					TX_buf[10]=DIAG_2.byte;
					TX_buf[11]=DIAG_3.byte;
					TX_buf[12]=master_address;
					TX_buf[13]=ident_H;
					TX_buf[14]=ident_L;
					createSD2(SA|0x80,ADDR|0x80,station|0x08,8);
					RW_State = PROFIBUS_WRITESTART;
					break;
				case DSAP_SET_PARAMETERS:
					createSC();
					RW_State = PROFIBUS_WRITESTART;
					if(ident_H==buf[6] && ident_L==buf[7]){
						if(HW_VERSION_HIGH==buf[9] && HW_VERSION_LOW==buf[10]){
							PRM_1.byte = buf[2];
							min_TSDR = buf[5];
							group_allocation = buf[8];
							DIAG_parameter_fault = 0;
							DIAG_parameter_request = 0;
							if(PRM_lock_request && !PRM_unlock_request && master_address == 0xFF){
								master_address = SA;
							}else if(PRM_lock_request && PRM_unlock_request){
								master_address = SA;
							}else if(PRM_unlock_request){
								master_address = 0xFF;
							}
							if(PRM_watchdog_on){
								DIAG_watchdog_on = 1;
								WD_threshold = buf[3]*buf[4]*10;
							}else{
								DIAG_watchdog_on = 0;
							}
						}else{
							DIAG_parameter_fault = 1;
						}
					}else{
						DIAG_parameter_fault = 1;
					}
					break;
				case DSAP_CHECK_CONFIG:
					createSC();
					DIAG_configuration_fault = 0;
					if(size==number_Of_Inputs+number_Of_Outputs+2){
						for(i=0;i<number_Of_Inputs;i++){
							if(buf[2+i]!= 0x10){
								DIAG_configuration_fault = 1;
							}
						}
						for(i=0;i<number_Of_Outputs;i++){
							if(buf[2+number_Of_Inputs+i] != 0x20){
								DIAG_configuration_fault = 1;
							}
						}
					}else{
						DIAG_configuration_fault = 1;
					}
					RW_State = PROFIBUS_WRITESTART;
					break;
				case DSAP_BROADCAST:
					break;
				default:
					createSD1(SA,ADDR,station|0x3);
					RW_State = PROFIBUS_WRITESTART;
					break;
			}
			break;
		case SSAP_DP_MS1:
			switch(buf[0]){
				case 0x33:
					switch(buf[2]){
						case 0x5F:
							DPV1_Write_request(&buf[3],size-3);
							break;
						case 0x5E:
							DPV1_Read_request(&buf[3],size-3);
							break;
						default:
							createSD1(SA,ADDR,station|0x3);
							RW_State = PROFIBUS_WRITESTART;
							break;
					}
					break;
				default:
					createSD1(SA,ADDR,station|0x3);
					RW_State = PROFIBUS_WRITESTART;
					break;
			}
			break;
		default:
			createSD1(SA,ADDR,station|0x3);
			RW_State = PROFIBUS_WRITESTART;
			break;
	}
}

byte CRC(const uint8_t *buf, const size_t size)
{
	uint8_t res = 0;
	uint8_t i;
	for(i=0;i<size;i++){
		res = res + buf[i];
	}
	return res;
}



void DPV1_Write_request(const uint8_t *buf, const size_t size){
	
	TX_buf[7]=SSAP_DP_MS1;
	TX_buf[8]=DSAP_DPV1_Read_Write_Res;
	TX_buf[9]=0x5F;
	TX_buf[10]=buf[0];
	TX_buf[11]=buf[1];
	TX_buf[12]=0x00;
		
	if((buf[0]&0x80) == 0x00){
		// Head Module Parameters || Virtual Parameters
		if(buf[1]<number_Of_Parameters && buf[2] == 1 && size == 4){
			PARAM[buf[1]]=buf[3];
		}else{
			// Error
			TX_buf[9]=0xDF;
			TX_buf[10]=0x80;
			TX_buf[11]=0xB2;
		}
	}else{
		// Module Parameters
		TX_buf[9]=0xDF;
		TX_buf[10]=0x80;
		TX_buf[11]=0xA9;
	}
		
	createSD2(SA|0x80,ADDR|0x80,station|0x08,6);
	RW_State = PROFIBUS_WRITESTART;
}

void DPV1_Read_request(const uint8_t *buf, size_t size){
	TX_buf[7]=SSAP_DP_MS1;
	TX_buf[8]=DSAP_DPV1_Read_Write_Res;
	TX_buf[9]=0x5E;
	TX_buf[10]=buf[0];
	TX_buf[11]=buf[1];
	TX_buf[12]=0x00;
	
	if((buf[0]&0x80) == 0x00){
		// Head Module Parameters || Virtual Parameters
		if(buf[1]<number_Of_Parameters && buf[2] == 1 && size == 3){
			TX_buf[12] = 1;
			TX_buf[13] = PARAM[buf[1]];
		}else{
			// Error
			TX_buf[9]=0xDE;
			TX_buf[10]=0x80;
			TX_buf[11]=0xB2;
		}
	}else{
		// Module Parameters
		TX_buf[9]=0xDE;
		TX_buf[10]=0x80;
		TX_buf[11]=0xA9;
	}
	createSD2(SA|0x80,ADDR|0x80,station|0x08,TX_buf[12]+6);
	RW_State = PROFIBUS_WRITESTART;
}

void FailSafe(){
	for(i=0;i<number_Of_Outputs;i++){
		OUT[i] = 0x00;
		STORED_OUT[i] = 0x00;
	}
}


