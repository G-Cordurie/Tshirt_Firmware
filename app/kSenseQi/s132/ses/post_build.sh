#!/bin/bash

nrfutil settings generate --family NRF52 --application $2/$1.hex --application-version 1 --bootloader-version 0 --bl-settings-version 1 $2/settings.hex