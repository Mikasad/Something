#ifndef __CANOPEN_DEF_H__
#define __CANOPEN_DEF_H__

#define UNS16_LE(v)  (v)
#define UNS32_LE(v)  (v)

/*OD_Macro_Define*/
#define SLAVE_NODE_NUM_MAX         0x7F
#define SLAVE_NODE_NUM_MIN         0x00
#define OD_ADDR_MAX                0xFFFF
#define OD_ADDR_MIN                0x0000
#define OD_NUM                     0
#define OD_FIRST_NUM               0
#define OD_LAST_NUM                OD_NUM
#define OBJ_ADD_NUM                0
#define CAN_ID_BIT                 0x7FF //11bit

#define BIT_VALID        1
#define BIT_INVALID      0
#define RTR_VALID        2
#define RTR_INVALID      0

#define STORE_PARA_SIGNATURE 0x65766173 //"e""v""a""s"
//Permison CanBandrate(Kbps)
#define CAN_BANDRATE_50K    50
#define CAN_BANDRATE_100K   100
#define CAN_BANDRATE_125K   125
#define CAN_BANDRATE_200K   200
#define CAN_BANDRATE_250K   250
#define CAN_BANDRATE_500K   500
#define CAN_BANDRATE_1000K  1000
//STD/EX
#define EX_CANID          0x1FFFFFFF
#define STD_CANID         0x000007FF
//LIMIT
#define LIMIT_COUNTER    0xFFFF
//Addr Index range
#define NO_USED_ADDR                    0x0000 //��ʹ��
#define STATIC_DATA_TYPE_BEGIN          0x0001 //��̬��������
#define STATIC_DATA_TYPE_END            0x001F 
#define COMPLEX_DATA_TYPE_BEGIN         0x0020 //������������
#define COMPLEX_DATA_TYPE_END           0x003F 
#define MANUFACTURER_COMPLEX_BEGIN      0x0040 //�����̶���ĸ�����������
#define MANUFACTURER_COMPLEX_END        0x005F 
#define DEVICE_STANDARD_STATIC_BEGIN    0x0060 //�豸�淶����ľ�̬��������
#define DEVICE_STANDARD_STATIC_END      0x007F 
#define DEVICE_STANDARD_COMPLEX_BEGIN   0x0080 //�豸�淶����ĸ�����������
#define DEVICE_STANDARD_COMPLEX_END     0x009F 
#define RESERVED1_BEGIN                 0x00A0 //����
#define RESERVED1_END                   0x0FFF 
#define COMUNICATION_STANDARD_BEGIN     0x1000 //ͨѶ�淶����(DS301)
#define COMUNICATION_STANDARD_END       0x1FFF 
#define DEVICE_MANUFACTURER_BEGIN       0x2000 //�豸�������������
#define DEVICE_MANUFACTURER_END         0x5FFF 
#define STANDARD_DEVICE_BEGIN           0x6000 //��׼�����豸�淶����
#define STANDARD_DEVICE_END             0x9FFF 
#define PORT_STANDARD_BEGIN             0xA000 //�ӿڹ淶˵������
#define PORT_STANDARD_END               0xBFFF 
#define RESERVED2_BEGIN                 0xC000 //����
#define RESERVED2_END                   0xFFFF 

#define PDO_MAP_BEGIN                   0x1A00 //PDOӳ�������ַ
#define PDO_MAP_END                     0x1A03 //
//Access Type Define
#define RW     0x00
#define WO     0x01
#define RO     0x02
#define CONST  0x04       //ֻ����Ϊ��������ֵ������NMT��ʼ��̬���ģ�����״̬���ɱ��
#define TO_BE_SAVE  0x08	
#define DCF_TO_SEND 0x10

#define SIZE_UINT8      1
#define SIZE_UINT16     2
#define SIZE_UINT32     4
#define SIZE_INTEGER8   1
#define SIZE_INTEGER16  2
#define SIZE_INTEGER32  4
#define SIZE_LONG       4
#define SIZE_FLOAT      4
//dataType define
#define BOLLEAN             0x00
#define NIL                 0x01
#define UNSIGNED8           0x02
#define UNSIGNED16          0x03
#define UNSIGNED24          0x04
#define UNSIGNED32          0x05
#define UNSIGNED40          0x06
#define UNSIGNED48          0x07
#define UNSIGNED56          0x08
#define INTEGER8            0x09
#define INTEGER16           0x0A
#define INTEGER24           0x0B
#define INTEGER32           0x0C
#define INTEGER40           0x0D
#define INTEGER48           0x0E
#define INTEGER56           0x0F
#define INTEGER64           0x10
#define REAL32              0x11
#define REAL364             0x12
#define STRUCT_OF           0x13
#define UNSIGNED8_STRING    0x14
#define UNSIGNED16_STRING   0x15

/*SDO_Define begin*/
#define SDO_COMUNICATION_PARA_INDEX   0x1200
//SDO������CCS(byte0��3λ)  ��վ->��վ
#define INIT_DOWNLOAD_REQUEST         0x1   //��ʼ������         
#define SEGMENT_DOWNLOAD_REQUEST      0x0   //�ֶ�����
#define INIT_UPLOAD_REQUEST           0x2   //��ʼ���ϴ�
#define SEGMENT_UPLOAD_REQUEST        0x3   //�ֶ��ϴ�
#define ABORT_TRANSER_REQUEST         0x4   //��ֹ�봫��
//SDO������SCS(byte0��3λ)  ��վ->��վ
#define INIT_DOWNLOAD_RESPOND         0x3
#define SEGMENT_DOWNLOAD              0x1
#define INIT_UPLOAD                   0x2
#define SEGMENT_UPLOAD                0x0
#define ABORT_TRANSER                 0x4
//Trans Type ExDefine
#define TYPE_INIT_DOWNLOAD_REQ        0x1
#define TYPE_SEGMENT_DOWNLOAD_REQ     0x0
#define TYPE_INIT_UPLOAD_REQ          0x2
#define TYPE_SEGMENT_UPLOAD_REQ       0x3
#define TYPE_ABORT_TRANSER_REQ        0x4
#define TYPE_SLAVE_ABORT_ACTIVE       0x5
//SDO SERVER_COB-ID
#define VALID_BIT_SET     0x80000000
#define VALID_BIT_CLR     0x7FFFFFFF
#define DYN_BIT_SET       0x40000000
#define DYN_BIT_CLR       0xBFFFFFFF
#define FRAME_BIT_SET     0x20000000
#define FRAME_BIT_CLR     0xDFFFFFFF

//SDO_cmd_bit
#define TRIG_BIT_SET     0x10 //����λ
#define TRIG_BIT_CLR     0xEF

#define E_BIT_SET        0x02 //�������� 0-�������� 1-���ٴ���
#define E_BIT_CLR        0xFD

#define S_BIT_SET        0x01 //�Ƿ�ָ�����ݳ��� 0-����δָ�� 1-����ָ��(��ʾ���ȴ���4Byte����ֶδ���)
#define S_BIT_CLR        0xFE

#define C_BIT_SET        0x01//�Ƿ��к����ֶ�����
#define C_BIT_CLR        0xFE
//SDO MessType
#define MessType_RSDO		 0x02
#define MessType_TSDO		 0x01

//�ڵ���SDO��������е�״̬
#define SDO_SERVER       0x01
#define SDO_CLIENT       0x02
#define SDO_UNKNOWN      0x03 

//SDO trans lenth
#define NORMAL_LEN_SDO   4   //SDOͨ�����ݳ���
#define SEGMENT_LEN_MAX_SDO 7 //�ֶδ��䵥���������
//SDO��ֹ����(δ�г�����ֹ����Ӧ����)
/* SDO Define end */

/*OD_Define���ʶ����ֵ�״̬*/
#define NO_RESULT                    0x0
#define OD_SUCCESSFUL 	             0x00000000
#define OD_READ_NOT_ALLOWED          0x06010001
#define OD_WRITE_NOT_ALLOWED         0x06010002
#define OD_NO_SUCH_OBJECT            0x06020000
#define OD_NOT_MAPPABLE              0x06040041
#define OD_LENGTH_DATA_INVALID       0x06070010
#define OD_NO_SUCH_SUBINDEX 	       0x06090011
#define OD_VALUE_RANGE_EXCEEDED      0x06090030 /* Value range test result */
#define OD_VALUE_TOO_LOW             0x06090031 /* Value range test result */
#define OD_VALUE_TOO_HIGH            0x06090032 /* Value range test result */
/* Others SDO abort codes 
 */
#define SDOABT_TOGGLE_NOT_ALTERNED   0x05030000
#define SDOABT_TIMED_OUT             0x05040000
#define SDOABT_OUT_OF_MEMORY         0x05040005 /* Size data exceed SDO_MAX_LENGTH_TRANSFER */
#define SDOABT_GENERAL_ERROR         0x08000000 /* Error size of SDO message */
#define SDOABT_LOCAL_CTRL_ERROR      0x08000021 



#define DEVICE_END_CODE_SUPPORT  //Support

#ifdef DEVICE_END_CODE_SUPPORT

#define INVER_BIT_NOT_CHANGE                  0x05030000   //��תλδ�仯
#define SERVER_CLIENT_CMD_UNKNOWN             0x05040001   //SDO�����ִ���
#define WRITE_ONLY_READ                       0x06010002   //��ͼд��ֻ������
#define NOT_EXIST_IN_OD                       0x06020000   //���󲻴���
#define CANNOT_MAP_PDO                        0x06040041   //�����ܱ�ӳ���PDO(�޸�PDOӳ�����ʱ)
#define DATATYPE_NOMATCH_SERVICE_LEN_NOMATCH  0x06070010   //�������ʹ���
#define SUB_INDEX_NOT_EXIST                   0x06090011   //������������
#define WRITE_PARA_TOOHIGH_DOWNLOAD_ONLY      0x06090031   //д�����ֵ̫��(������)
#define WRITE_PARA_TOOLOW_DOWNLOAD_ONLY       0x06090032   //д�����ֵ̫С(������)
#define GENERAL_ERROR                         0x08000000   //�������
#define ACCESS_FAILED_DUE_TO_HARDWARE         0x06060000   //Ӳ�������µķ���ʧ��
#define LENGTH_NUMBER_BEYOND_PDO_LEN          0x06040042   //
#define GEERAL_PARA_INCOMPATIBILITY_REASON    0x06040043   //������������ݶԭ��
#else
#define INVER_BIT_NOT_CHANGE                  0x05030000   //��תλδ�仯
#define SDO_AGREEMENT_OUTTIME                 0x05040000   //SDOЭ�鳬ʱ
#define SERVER_CLIENT_CMD_UNKNOWN             0x05040001   //�ͻ���/������������Ч��δ֪
#define INVALID_BLOCK_SIZE                    0x05040002   //��Ч�Ŀ��С
#define INVALID_SERIAL_NUMBER                 0x05040003   //��Ч�����к�
#define CRC_ERR                               0x05040004   //CRC����(�������)
#define OUT_OF_MEMORY                         0x05040005   //�ڴ治��
#define ACCESS_UNSUPPORTEED                   0x06010000   //��֧�ֵķ��ʶ���
#define READ_ONLY_WRITE                       0x06010001   //��ͼ��ȡֻд����
#define WRITE_ONLY_READ                       0x06010002   //��ͼд��ֻ������
#define NOT_EXIST_IN_OD                       0x06020000   //���󲻴�����OD
#define CANNOT_MAP_PDO                        0x06040041   //�����ܱ�ӳ���PDO
#define LENGTH_NUMBER_BEYOND_PDO_LEN          0x06040042   //���󳤶Ⱥ���������PDO����
#define GEERAL_PARA_INCOMPATIBILITY_REASON    0x06040043   //������������ݶԭ��
#define DEVICE_INTERNAL_INCOMPATIBILITY       0x06040047   //�豸�ڲ�������
#define DATATYPE_NOMATCH_SERVICE_LEN_NOMATCH  0x06070010   //�������Ͳ�ƥ�䣬���񳤶Ȳ�����ƥ��
#define DATATYPE_NOMATCH_SERVICE_LEN_TOOHIGH  0x06070012   //�������Ͳ�ƥ�䣬���񳤶Ȳ���̫��
#define DATATYPE_NOMATCH_SERVICE_LEN_TOOLOW   0x06070013   //�������Ͳ�ƥ�䣬���񳤶Ȳ���̫С
#define SUB_INDEX_NOT_EXIST                   0x06090011   //������������
#define INVALID_PARA_DOWNLOAD_ONLY            0x06090030   //��Ч�Ĳ���ֵ(������)
#define WRITE_PARA_TOOHIGH_DOWNLOAD_ONLY      0x06090031   //д�����ֵ̫��(������)
#define WRITE_PARA_TOOLOW_DOWNLOAD_ONLY       0x06090032   //д�����ֵ̫��(������)
#define MAXIMUM_VALUE_LESS_THAN_MINIMUM       0x06090036   //���ֵС����Сֵ
#define RESOURCE_NOT_ACCESS_SDO_CONNECT       0x060A0023   //��Դ�����ã�SDO����
#define GENERAL_ERROR                         0x08000000   //�������
#define DATA_CANNOT_TRANS_STORED_TO_APP       0x08000020   //���ݲ��ܱ�����򱣴浽Ӧ�ó���
#define DATA_CANNOT_TRANS_STORED_BECAUSE_LOCAL_CTR 0x08000021   //���ݲ��ܱ�����򱣴浽Ӧ�ó������ڱ��صĿ���
#define DATA_CANNOT_TRANS_STORED_BECAUSE_PRESENT_DEVICE_STATE 0x08000022   //���ݲ��ܱ�����򱣴浽Ӧ�ó�����Ϊ��ǰ�豸״̬
#define OD_DYNAMIC_GENERATION_FAIL_OR_OD_NOTEXIST 0x08000023   //�����ֵ�Ķ�̬����ʧ�ܻ��޶����ֵ�Ĵ���(��������ֵ��Ǵ��ļ����ɣ��������ļ��ĳ���������ʧ��)
#define NO_DATA_AVAILABLE                     0x080000241   //�޿�������

#endif
/*PDO_Define*/
#define PDO_COMUNICATION_PARA_INDEX   0x1800  //ͨѶ��������
#define PDO_MAPPING_PARA_INDEX        0x1A00  //ӳ���������
//����PDOӳ��
#define PDO_ALLOW 1
#define PDO_BAN   0
//TPDO/RPDO��ʶ��
#define PDO_EXIST_BIT_SET       0x7FFFFFFF
#define PDO_EXIST_BIT_CLR       0x80000000
#define PDO_EXIST_BIT_DEFLT     PDO_EXIST_BIT_SET

#define PDO_ALLOW_RTR_BIT_SET   0xBFFFFFFF
#define PDO_ALLOW_RTR_BIT_CLR   0x40000000
#define PDO_ALLOW_RTR_DEFLT     PDO_ALLOW_RTR_BIT_SET

#define PDO_20A_BIT_SET         0xDFFFFFFF
#define PDO_20B_BIT_SET         0x20000000
#define PDO_ID_STANDARD_DEFLT   PDO_20A_BIT_SET
//PDO��������
#define SYNC_NOCYCLE      0     //ͬ�� ��ѭ��[�յ�һ��ͬ���źź�ͽ���һ��PDO����]
#define SYNC_CYCLE_BEGIN  1     //ͬ�� ѭ����ʼ[�յ�N��ͬ���źź����һ��PDO����]
#define SYNC_CYCLE_END    240   //ͬ�� ѭ������
#define RESERVED_BEGIN    241   //������ʼ
#define RESERVED_END      251   //��������
#define SYNC_RTR_ONLY     252   //ͬ�� ��Զ�̴���
#define ASYNC_TR_ONLY     253   //�첽 ��Զ�̴���[�յ�һ��Զ��֡�����һ��PDO����]
#define ASYNC_TRIGGER_A   254   //�첽 �������ض��¼�
#define ASYNC_TRIGGER_B   255   //�첽 �豸��Э���ض��¼�

#define PDO_MAP_NUMBER_MAX 0x8 //0x40
//PDO MAP PARA QUICK INDEX
#define MAP_INDEX_OD_ADDR 0     //ӳ�����-OD��ַ
#define MAP_INDEX_OD_SUB  1     //ӳ�����-OD��ַ��Ӧ�±�
//PDO trans lenth
#define PDO_TRANSLEN_MAX  8

/* constantes used in the different state machines */
/* ----------------------------------------------- */
/* Must not be modified */
#define state1  0x01
#define state2  0x02
#define state3  0x03
#define state4  0x04
#define state5  0x05
#define state6  0x06
#define state7  0x07
#define state8  0x08
#define state9  0x09
#define state10 0x0A
#define state11 0x0B

/*DS402*/
//Motor Type
#define NON_STANDERD_MOTOR            0x0000  
#define PHASE_MODULATED_DC_MOTOR      0x0001
#define FRAQ_CONTROLLED_DC_MOTOR      0x0002 
#define PM_SYNC_MOTOR                 0x0003 
#define FC_SYNC_MOTOR                 0x0004 
#define SWITCH_RELUCTANCE_MOTOR       0x0005 
#define WOUND_ROTOR_INDUCTION_MOTOR   0x0006 
#define SQUIREL_CAGE_INDUCTION_MOTOR  0x0007 
#define STEPPER_MOTOR                 0x0008 
#define MICRO_STEP_STEPPER_MOTOR      0x0009 
#define SINUSOIDAL_PM_BL_MOTOR        0x000A 
#define TRAPEZOIDAL_PM_BL_MOTOR       0x000B 
#define MOTOR_TYPE_RESERVEDS          0x000C 
//Supported Driver mode
#define DRIVER_PP          (1<<0)
#define DRIVER_VL          (1<<1)
#define DRIVER_PV          (1<<2)
#define DRIVER_TQ          (1<<2)
#define DRIVER_RESERVED_0  (0<<3)
#define DRIVER_HM          (1<<4)
#define DRIVER_IP          (1<<5)
#define DRIVER_RESERVED_1  ((0x00<<7)|(0<<6))
#define DRIVER_MANUFACTUREER_SPECIFIC  (0x0006<<16)
//Motion profile type
#define LINEAR_RAMP                 0x0000
#define SIN2_RAMP                   0x0001
#define JERK_FREE_RAMP              0x0002
#define JERK_LIMITED_RAMP           0x0003

#endif
/* This is End */
