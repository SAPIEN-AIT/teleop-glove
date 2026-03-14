#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "mdns.h"
#include "wifi.h"
#include "REST_API.h"
#include "ICM20948.h"
#include "communication.h"
#include "JointAngle.h"
#include "Quaternion.h"
#include "config.h"

extern "C" void app_main(void)
{
    WiFiManager wifi("YourSSID", "YourPassword");
    wifi.connect();
    wifi.waitConnected();
    
    
    mdns_init();
    mdns_hostname_set("mina.glove");
    mdns_instance_name_set("ESP32 Glove Controller");
}
