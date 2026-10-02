idf_component_register(SRCS "test_on9rstore.cpp"
                       REQUIRES on9ringstore fatfs esp_timer esp_partition wear_levelling unity
                       WHOLE_ARCHIVE)
