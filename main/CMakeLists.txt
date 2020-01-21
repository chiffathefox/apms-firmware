idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "")

target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "certs/aws-root-ca.pem" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "certs/certificate.pem.crt" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "certs/private.pem.key" TEXT)

