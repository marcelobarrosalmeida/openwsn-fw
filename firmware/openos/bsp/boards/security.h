#ifndef __SECURITY_H__
#define __SECURITY_H__

/**
\addtogroup BSP
\{
\addtogroup security
\{

\brief Cross-platform declaration "security" bsp module.

\author Marcelo Barros <marcelobarrosalmeida@gmail.com>, May 2014.
*/


 
//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

enum IEEE802154_sec_levels {   
    // ENC (encrypt) x MIC (message integrity code)
    IEEE802154_SEC_LEVEL_NONE        = 0,
    IEEE802154_SEC_LEVEL_MIC32       = 1,
    IEEE802154_SEC_LEVEL_MIC64       = 2,
    IEEE802154_SEC_LEVEL_MIC128      = 3,
    IEEE802154_SEC_LEVEL_ENC         = 4,
    IEEE802154_SEC_LEVEL_ENC_MIC32   = 5,
    IEEE802154_SEC_LEVEL_ENC_MIC64   = 6,
    IEEE802154_SEC_LEVEL_ENC_MIC128  = 7
};
void security_init();
uint8_t security_get_level();
uint8_t security_decrypt(OpenQueueEntry_t* msg, ieee802154_sec_hdr_t *sec_hdr);
uint8_t security_encrypt(OpenQueueEntry_t* msg, ieee802154_sec_hdr_t *sec_hdr);
/**
\}
\}
*/

#endif /* __SECURITY_H__ */

