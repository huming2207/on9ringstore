set(on9rstore_requires
        "fatfs" "freertos"
)

set(on9rstore_priv_requires
        "esp_timer" "esp_system"
)

if(CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH)
    list(APPEND on9rstore_priv_requires "esp_partition" "espcoredump")
endif()

idf_component_register(
        SRCS
            "on9rstore.cpp"
            "on9rstore_entry.cpp"
            "on9rstore_io.cpp"
            "on9rstore_manifest.cpp"
            "on9rstore_segment.cpp"

        INCLUDE_DIRS
            "."

        REQUIRES
            ${on9rstore_requires}

        PRIV_REQUIRES
            ${on9rstore_priv_requires}
)
