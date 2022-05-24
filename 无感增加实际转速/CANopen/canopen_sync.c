#include "canopen_sync.h"


/*------------------------------------------------
Function:�����Ƿ�����ͬ����Ϣ
Input   :d
Output  :No
Explain :No
------------------------------------------------*/
void StopSync(CO_Data *d)
{
    RegisterSetOnDentry_Callback(&CANopen_Drive,0x1005,0,NULL);//COB-IDͬ����Ϣ(SYNC�����COB-ID)
    RegisterSetOnDentry_Callback(&CANopen_Drive,0x1006,0,NULL);//ͨ��ѭ�����ڣ����Ϊ0�����SYNC
}
/*------------------------------------------------
Function:��ʼ��SYNC
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void ResrtSync(void)
{

}
/*------------------------------------------------
Function:����ͬ������
Input   :d
Output  :No
Explain :No
------------------------------------------------*/
unsigned char proceedSYNC(CO_Data *d)
{
    unsigned char res;

    (*d->post_sync)(d);

    if(!d->CurrentCommunicationState.csPDO)
        return 0;

    RPDO_SYNC_SIGN |= RPDO_SYNC_READY;

    res = _sendPDOevent(d,pRpdoDirectPar,1);

    (*d->post_TPDO)(d);

    return res;
}
/*------------------------------------------------
Function:���յ�ͬ�����Ļص�����
Input   :d
Output  :No
Explain :No
------------------------------------------------*/
void _post_sync(CO_Data* d) {}
/*------------------------------------------------
Function:tpdo���Ļص�����
Input   :d
Output  :No
Explain :No
------------------------------------------------*/
void _post_TPDO(CO_Data* d) {}
