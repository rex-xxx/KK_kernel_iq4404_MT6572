#ifndef TOUCHPANEL_H  
#define TOUCHPANEL_H  
  
#define MS_TS_MSG21XX_X_MAX                480  
#define MS_TS_MSG21XX_Y_MAX                854  
  
#define TPD_HAVE_BUTTON  
#define MAX_TOUCH_FINGER	2
#define REPORT_PACKET_LENGTH	8
  
  
#define TPD_BUTTON_HEIGHT   854  
#define TPD_KEY_COUNT           3  
#define TPD_KEYS                {KEY_BACK, KEY_HOME, KEY_MENU}  
#define TPD_KEYS_DIM            {{40,880,80,50},{200,880,80,50},{360,880,80,50}}  
  
#endif