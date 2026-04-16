#include "uartx_callback.h"
#include "data_task.h"

//���ڽ������ݵĽṹ��
RECEIVE_DATA Receive_Data;

/**************************************************************************��
Functionality: This function receives serial commands to control the movement of a robot.
Input Parameters:
Which serial port (e.g.,USART1,USART3,etc.)
The received data from the serial port
Return Value: None
Author: WHEELTEC
�������ܣ����մ����������ڿ��ƻ������˶�
��ڲ�������һ�����ڣ����յ�������
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static void UartxControll_Callback(USART_TypeDef* USARTx,uint8_t Usart_Receive)
{
	static uint16_t Count = 0;
	static USART_TypeDef* Last_Uartx;
	
	//��ֹ�മ��ͬʱ���ú���,һ��ʱ��ֻ����һ���ӿڶԻ����˽��п���.
	if( USARTx != Last_Uartx && Count!=0 )
	{
		Count = 0;
	}
	Last_Uartx = USARTx;
	
	//Fill the array with serial data
	//����������������
	Receive_Data.buffer[Count]=Usart_Receive;

	// Ensure that the first data in the array is FRAME_HEADER
	//ȷ�������һ������ΪFRAME_HEADER
	if(Usart_Receive == FRAME_HEADER||Count>0)
		Count++;
	else
		Count=0;

	if (Count == 11) //Verify the length of the packet //��֤���ݰ��ĳ���
	{
		Count=0; //Prepare for the serial port data to be refill into the array //Ϊ����������������������׼��
		if(Receive_Data.buffer[10] == FRAME_TAIL) //Verify the frame tail of the packet //��֤���ݰ���֡β
		{
			//Data exclusionary or bit check calculation, mode 0 is sent data check
			//�������λУ����㣬ģʽ0�Ƿ�������У��
			if(Receive_Data.buffer[9] ==Check_BCC(Receive_Data.buffer,9))
			{	
				//���ڡ�CAN�������ʧ��������
				robot_control.command_lostcount = 0;

				//========== 发送ACK回显 ==========
				//将接收到的11字节命令帧原样返回，作为收到命令的应答
				for(int i=0; i<11; i++)
				{
					if(USARTx == USART1)
						uart1_send(Receive_Data.buffer[i]);
					else if(USARTx == USART3)
						uart3_send(Receive_Data.buffer[i]);
				}
				//===================================

				if(Receive_Data.buffer[1]==0)
				{
					//���ݴ��ڵĲ�ͬ��ȷ����ʲô���Ʒ�ʽ
					       if(USARTx==USART1) Set_Control_Mode(_USART_Control);
					else if( USARTx==USART3 ) Set_Control_Mode(_ROS_Control);
					
					charger.AllowRecharge = 0;
					//Calculate the target speed of three axis from serial data, unit m/s
					//�Ӵ�������������Ŀ���ٶȣ� ��λm/s
					robot_control.Vx = XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
					robot_control.Vy = XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
					robot_control.Vz = XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
					
					#if defined AKM_CAR 
					//Convert the Z-axis speed to the Ackerman left front wheel steering angle \
					for an Ackerman-type vehicle, and then input the calculated angle into the \
					inverse kinematics to control the vehicle.	
					//���������ͽ�Z���ٶ�ת��Ϊ��������ǰ��ת��,���������뵽�˶�ѧ����п���С��.
					robot_control.Vz = Akm_Vz_to_Angle( robot_control.Vx , robot_control.Vz );
					#endif
				}
				else if( Receive_Data.buffer[1]==1 || Receive_Data.buffer[1]==2 )
				{
					//�����Զ��س�
					charger.AllowRecharge = 1;
					
					//������������ͨ����־λ�жϵ�������
					#if defined AKM_CAR
						if(Receive_Data.buffer[1]==2) charger.NavWalk=1; 
						else charger.NavWalk=0;
					
					//����������ͨ�������ж��Ƿ�ʹ�õ�������
					#else
						if(Receive_Data.buffer[1]==1 && charger.RED_STATE==0) charger.NavWalk = 1; 
					#endif
					
					//Calculate the target speed of three axis from serial data, unit m/s
					//�Ӵ�������������Ŀ���ٶȣ���λm/s
					charger.Up_MoveX = XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
					charger.Up_MoveY = XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
					charger.Up_MoveZ = XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
					
					#if defined AKM_CAR 
					//Convert the Z-axis speed to the Ackerman letf front wheel steering angle \
					for an Ackerman-type vehicle, and then input the calculated angle into the \
					inverse kinematics to control the vehicle.	
					//���������ͽ�Z���ٶ�ת��Ϊ��������ǰ��ת��,���������뵽�˶�ѧ����п���С��.
					charger.Up_MoveZ = Akm_Vz_to_Angle( charger.Up_MoveX , charger.Up_MoveZ );
					#endif

				}
				else if( Receive_Data.buffer[1]==3 )
				{
					//Set the speed of the infrared interconnection, unit m/s
					//���ú���Խӵ��ٶȴ�С����λm/s
					charger.Dock_MoveX = XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
					charger.Dock_MoveY = XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
					charger.Dock_MoveZ = XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
				}
			}
		}
	}
}


/**************************************************************************
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
�������ܣ�����1�����ж�
��ڲ�������
�� �� ֵ����
**************************************************************************/
int USART1_IRQHandler(void)
{
    u8 Usart_Receive;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
    {
        Usart_Receive = USART_ReceiveData(USART1);//Read the data //��ȡ����
		
		// Data is not processed until 25 seconds after startup
		//���� CONTROL_DELAY ��ǰ����������
        if(SysVal.Time_count<CONTROL_DELAY)
            return 0;	//ǰ�ڲ������ж�

		//���������˴��ڿ�������
        UartxControll_Callback(USART1,Usart_Receive);
    }
    return 0;
}
/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
�������ܣ�����4�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int UART4_IRQHandler(void)
{
    int Usart_Receive;
    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
    {
        static u8 Flag_PID,i,j,Receive[50],Last_Usart_Receive;
        static float Data;

        Usart_Receive=UART4->DR; //Read the data //��ȡ����
		
		//����������λϵͳ,����BootLoader��¼����
        _System_Reset_(Usart_Receive);

		// Data is not processed until 10 seconds after startup
		//����10��ǰ����������
        if(SysVal.Time_count<CONTROL_DELAY)
            return 0;
		
		//����APP���ӺͶϿ�ʱ��ATָ��,��ֹ����
        if(AT_Command_Capture(Usart_Receive)) return 1;

		//10 seconds after startup, press the forward button of APP to enter APP control mode
		//The APP controls the flag position 1 and the other flag position 0
		//����10��֮�󣬰���APP��ǰ��������APP����ģʽ
        if(Usart_Receive==0x41&&Last_Usart_Receive==0x41&&(Get_Control_Mode(_APP_Control))==0)
            Set_Control_Mode(_APP_Control);

        Last_Usart_Receive=Usart_Receive;

        //Enter the APP steering control interface
        //����APPת����ƽ���
        if(Usart_Receive==0x4B)
            appkey.TurnPage = 1;
        else if(Usart_Receive==0x49||Usart_Receive==0x4A)
            appkey.TurnPage = 0;

        if( 0 == appkey.TurnPage )
        {
            //App rocker control interface command
            //APPҡ�˿��ƽ�������
            if(Usart_Receive>=0x41&&Usart_Receive<=0x48)
            {
                appkey.DirectionFlag=Usart_Receive-0x40;
            }
            else	if(Usart_Receive<=8)
            {
                appkey.DirectionFlag=Usart_Receive;
            }
            else  appkey.DirectionFlag=0;
        }
        else if( 1 == appkey.TurnPage )
        {
            //APP steering control interface command
            //APPת����ƽ�������
            if     (Usart_Receive==0x43) appkey.TurnFlag = 2; //Right rotation //����ת
            else if(Usart_Receive==0x47) appkey.TurnFlag = 1; //Left rotation  //����ת
            else                         appkey.TurnFlag = 0;

            if     (Usart_Receive==0x41||Usart_Receive==0x45) appkey.DirectionFlag=Usart_Receive-0x40;
            else  appkey.DirectionFlag=0;
        }

        if(Usart_Receive==0x58)  robot_control.rc_speed+=100; //Accelerate the keys, +100mm/s //���ٰ�����+100mm/s
        if(Usart_Receive==0x59)  robot_control.rc_speed-=100; //Slow down buttons,   -100mm/s //���ٰ�����-100mm/s

        // The following is the communication with the APP debugging interface
        //��������APP���Խ���ͨѶ
        if(Usart_Receive==0x7B) Flag_PID=1;   //The start bit of the APP parameter instruction //APP����ָ����ʼλ
        if(Usart_Receive==0x7D) Flag_PID=2;   //The APP parameter instruction stops the bit    //APP����ָ��ֹͣλ
		
        if( Usart_Receive=='b' ) charger.AllowRecharge = !charger.AllowRecharge;
        else if(Usart_Receive=='m'  )
        {
            oled.page++;
            if(oled.page>oled.MAX_PAGE) oled.page=1;
        }

        if(Flag_PID==1) //Collect data //�ɼ�����
        {
            Receive[i]=Usart_Receive;
            i++;
        }
        if(Flag_PID==2) //Analyze the data //��������
        {
            if(Receive[3]==0x50) 	 appkey.ParamSendflag = 1; //�������ݵ�app��ʾ
            else if( Receive[3]==0x57 ) appkey.ParamSaveFlag = 1; //���������stm32 flash
            else  if(Receive[1]!=0x23)
            {
                for(j=i; j>=4; j--)
                {
                    Data+=(Receive[j-1]-48)*pow(10,i-j);
                }
                switch(Receive[1])
                {
					case 0x30: robot_control.rc_speed = Data ; break; //�޸Ļ����˵�ң���ٶ�
					//PID��������
					case 0x31:
						Set_Robot_PI_Param(Data,-1);/* ʹ��-1������Ӧ�Ĳ��������� */
						break;
					case 0x32:
						Set_Robot_PI_Param(-1,Data);/* ʹ��-1������Ӧ�Ĳ��������� */
						break;
					
					case 0x33: robot_control.smooth_MotorStep = Data/1000 ; break; 
					case 0x34: robot_control.smooth_ServoStep = Data ;  break;         
					case 0x35: Akm_Servo.Max=Data;  break;        
					case 0x36: Akm_Servo.Min=Data; break;                
					case 0x37: Akm_Servo.Mid=Data; break;                
					case 0x38: robot_control.LineDiffParam = Data; break;       
                }
            }
            else if( Receive[1]==0x23 ) //APP�ϵ���������������ݡ���������
            {
                float num;
                u8 dataIndex=0;
                float dataArray[9];

                if( i<=50 ) //�����ڿɽ��ܷ�Χ
                {
                    Receive[i]='}'; //����֡β

                    for(u8 kk=0; Receive[kk]!='}'; kk++)
                    {
                        if( Receive[kk]>='0' && Receive[kk]<='9' )
                        {
                            num = num*10 + ( Receive[kk] - '0' );
                        }
                        else if( Receive[kk]==':' )
                        {
                            dataArray[dataIndex++] = num;
                            num = 0;
                        }

                    }
                    //�������һ������
                    dataArray[dataIndex] = num;

                    //���ݸ�ֵ
                    robot_control.rc_speed = dataArray[0];

                    //kp��ki
                    Set_Robot_PI_Param(dataArray[1],dataArray[2]);
					
                    //�ٶ�ƽ��ϵ��
                    robot_control.smooth_MotorStep = dataArray[3]/1000;
                    robot_control.smooth_ServoStep = dataArray[4];

                    //������ת������
                    Akm_Servo.Max = dataArray[5];
                    Akm_Servo.Min = dataArray[6];
                    Akm_Servo.Mid = dataArray[7];
					
					//��ƫϵ��
					robot_control.LineDiffParam = dataArray[8];
					
                    //�����������޸ĵ����ݵ�app��ʾ
                    appkey.ParamSendflag=1;
                }
            }

            //Relevant flag position is cleared
            //��ر�־λ����
            Flag_PID=0;
            i=0;
            j=0;
            Data=0;
            memset(Receive, 0, sizeof(u8)*50); //Clear the array to zero//��������
        }
		
		//��ң�ص��ٶ������ֵ����Сֵ����
		robot_control.rc_speed = target_limit_float(robot_control.rc_speed,0,robot_control.limt_max_speed*1000);
		
		//���ƾ�ƫ������Χ��0~100
		robot_control.LineDiffParam = robot_control.LineDiffParam > 100 ? 100 : robot_control.LineDiffParam;
    }
    return 0;
}
/**************************************************************************
Function: Serial port 3 receives interrupted
Input   : none
Output  : none
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int USART3_IRQHandler(void)
{
    u8 Usart_Receive;

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
    {
        Usart_Receive = USART_ReceiveData(USART3);//Read the data //��ȡ����

		// Data is not processed until 10 seconds after startup
		//����10��ǰ����������
        if(SysVal.Time_count<CONTROL_DELAY)
            return 0;

		//���������˴��ڿ�������
        UartxControll_Callback(USART3,Usart_Receive);
    }
    return 0;
}

/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
�������ܣ�����λ���������ĸ�8λ�͵�8λ�������ϳ�һ��short�����ݺ�������λ��ԭ����
��ڲ�������8λ����8λ
����  ֵ��������X/Y/Z���Ŀ���ٶ�
**************************************************************************/
static float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
    //Data conversion intermediate variable
    //����ת�����м����
    short transition;

    //����8λ�͵�8λ���ϳ�һ��16λ��short������
    //The high 8 and low 8 bits are integrated into a 16-bit short data
    transition=((High<<8)+Low);
    return
        transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s->m/s //��λת��, mm/s->m/s
}


//����ATָ��ץ������ָֹ����ŵ�����������������ͨ��
static u8 AT_Command_Capture(u8 uart_recv)
{
    /*
    ��������ʱ���͵��ַ���00:11:22:33:44:55Ϊ������MAC��ַ
    +CONNECTING<<00:11:22:33:44:55\r\n
    +CONNECTED\r\n
    ��44���ַ�

    �����Ͽ�ʱ���͵��ַ�
    +DISC:SUCCESS\r\n
    +READY\r\n
    +PAIRABLE\r\n
    ��34���ַ�
    \r -> 0x0D
    \n -> 0x0A
    */

    static u8 pointer = 0; //��������ʱָ���¼��
    static u8 bt_line = 0; //��ʾ�����ڵڼ���
    static u8 disconnect = 0;
    static u8 connect = 0;

    //�Ͽ�����
    static char* BlueTooth_Disconnect[3]= {"+DISC:SUCCESS\r\n","+READY\r\n","+PAIRABLE\r\n"};

    //��ʼ����
    static char* BlueTooth_Connect[2]= {"+CONNECTING<<00:00:00:00:00:00\r\n","+CONNECTED\r\n"};


    //�����ʶ������ʼ����(ʹ��ʱҪ-1)
    if(uart_recv=='+')
    {
        bt_line++,pointer=0; //�յ���+������ʾ�л�������
        disconnect++,connect++;
        return 1;//ץ������ֹ����
    }

    if(bt_line!=0)
    {
        pointer++;

        //��ʼ׷�������Ƿ���϶Ͽ�������������ʱȫ�����Σ�������ʱȡ������
        if(uart_recv == BlueTooth_Disconnect[bt_line-1][pointer])
        {
            disconnect++;
            if(disconnect==34) disconnect=0,connect=0,bt_line=0,pointer=0;
            return 1;//ץ������ֹ����
        }

        //׷���������� (bt_line==1&&connect>=13)����������MAC��ַ��ÿһ������MAC��ַ������ͬ������ֱ�����ι�ȥ
        else if(uart_recv == BlueTooth_Connect[bt_line-1][pointer] || (bt_line==1&&connect>=13) )
        {
            connect++;
            if(connect==44) connect=0,disconnect=0,bt_line=0,pointer=0;
            return 1;//ץ������ֹ����
        }

        //��ץ���ڼ��յ��������ֹͣץ��
        else
        {
            disconnect = 0;
            connect = 0;
            bt_line = 0;
            pointer = 0;
            return 0;//�ǽ�ֹ���ݣ����Կ���
        }
    }

    return 0;//�ǽ�ֹ���ݣ����Կ���
}



//����λ��BootLoader����
static void _System_Reset_(u8 uart_recv)
{
    static u8 res_buf[5];
    static u8 res_count=0;

    res_buf[res_count]=uart_recv;

    if( uart_recv=='r'||res_count>0 )
        res_count++;
    else
        res_count = 0;

    if(res_count==5)
    {
        res_count = 0;
        //���ܵ���λ������ĸ�λ�ַ���reset����ִ��������λ
        if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
        {
            NVIC_SystemReset();//����������λ����λ��ִ�� BootLoader ����
        }
    }
}

