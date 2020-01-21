#
# Component Makefile
#

CFLAGS += -Wall -Wextra
COMPONENT_EMBED_TXTFILES := \
    certs/aws-root-ca.pem certs/certificate.pem.crt certs/private.pem.key


$(COMPONENT_PATH)/certs/certificate.pem.crt \
    $(COMPONENT_PATH)/certs/private.pem.key \
    $(COMPONENT_PATH)/certs/aws-root-ca.pem:
	@echo "Missing PEM file $@. Please provide all the certificates" \
	    "necessary to identify this 'thing' to AWS IoT."
	exit 1
