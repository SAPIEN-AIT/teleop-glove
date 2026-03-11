# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS
        "main.c"
       
        "ICM20948.cpp"
        "JointAngle.cpp"
        "Quaternion.cpp"
        "REST_API.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp_driver_i2c
        esp_http_server
        esp_wifi
        esp_netif
        nvs_flash
)