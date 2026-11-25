# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(SRCS "main.c"
                            "EPD.c"
                            "EPD_GUI.c"
                            "EPD_SPI.c"
                            "wifi_ota.c"
                            "max17055.c"
   EMBED_TXTFILES server_root_cert.pem)
