var group__i2c__driver =
[
    [ "_i2c_master_config", "struct__i2c__master__config.html", [
      [ "baudRate_Bps", "struct__i2c__master__config.html#a282b6a9bd8e98bf35e8183665488d98c", null ],
      [ "enableMaster", "struct__i2c__master__config.html#a7a58fd313d0615ac1ca70d2b4f17ae9a", null ],
      [ "glitchFilterWidth", "struct__i2c__master__config.html#aa4b37238d6ae63282caf0e0d3ac6b018", null ]
    ] ],
    [ "_i2c_slave_config", "struct__i2c__slave__config.html", [
      [ "addressingMode", "struct__i2c__slave__config.html#aa8534ccd91e4ba5c073cf7c3a7bcd2cf", null ],
      [ "enableBaudRateCtl", "struct__i2c__slave__config.html#a40977851284d289e40482d8970d8a7a0", null ],
      [ "enableGeneralCall", "struct__i2c__slave__config.html#a84c7c8599005d18bfc2d59eda0f819bf", null ],
      [ "enableSlave", "struct__i2c__slave__config.html#ad720331a891010209a1209585056a2b7", null ],
      [ "enableWakeUp", "struct__i2c__slave__config.html#a0c8e49d73b44eb374c6ad9eaa9d28085", null ],
      [ "sclStopHoldTime_ns", "struct__i2c__slave__config.html#ab8d990d75d9e07d2db5afd85713cfb2e", null ],
      [ "slaveAddress", "struct__i2c__slave__config.html#a60784d4b0182f877068255d5c2f54732", null ],
      [ "upperAddress", "struct__i2c__slave__config.html#a0736edacc0e7833facc613181209cb7d", null ]
    ] ],
    [ "_i2c_master_transfer", "struct__i2c__master__transfer.html", [
      [ "data", "struct__i2c__master__transfer.html#a89a3ac8137ef778590d0ab3581187a60", null ],
      [ "dataSize", "struct__i2c__master__transfer.html#a2e4cb139c00e6dd854232d4b2bfcf87a", null ],
      [ "direction", "struct__i2c__master__transfer.html#a7bd3afcf89b78d9397b2f0c55f9deb27", null ],
      [ "flags", "struct__i2c__master__transfer.html#a84861e140902bd67ae901b2f4c53bfc0", null ],
      [ "slaveAddress", "struct__i2c__master__transfer.html#aa6d93caa78fc9bf0f0aac10f66b8eba1", null ],
      [ "subaddress", "struct__i2c__master__transfer.html#a7a4f43a16e63d9f286bc41b6255b5e3f", null ],
      [ "subaddressSize", "struct__i2c__master__transfer.html#a810393f12a8bd6360e04b79ea83fcf3b", null ]
    ] ],
    [ "_i2c_master_handle", "struct__i2c__master__handle.html", [
      [ "completionCallback", "struct__i2c__master__handle.html#a0c6862e22cf2047661ce05975756093a", null ],
      [ "state", "struct__i2c__master__handle.html#ade67a2d6339836711ee255a18405e750", null ],
      [ "transfer", "struct__i2c__master__handle.html#a403d5da02efab45e1ebb8bef2ae063d5", null ],
      [ "transferSize", "struct__i2c__master__handle.html#aab5058b059faf2268848645a1b204cfe", null ],
      [ "userData", "struct__i2c__master__handle.html#a862e1a312eb8b2bd537f1a22a48f455f", null ]
    ] ],
    [ "_i2c_slave_transfer", "struct__i2c__slave__transfer.html", [
      [ "completionStatus", "struct__i2c__slave__transfer.html#a5c09cb680bd188a4176823b6dfa21335", null ],
      [ "data", "struct__i2c__slave__transfer.html#a0d4fde45f82fa9dfdb7d13d4e123a894", null ],
      [ "dataSize", "struct__i2c__slave__transfer.html#a59fbecd8b66464de8013c39e40eb4b30", null ],
      [ "event", "struct__i2c__slave__transfer.html#a461d482d4a91765e8820b55b17e7e53c", null ],
      [ "transferredCount", "struct__i2c__slave__transfer.html#aae53c04dd44495a5f8cf8813e715a53e", null ]
    ] ],
    [ "_i2c_slave_handle", "struct__i2c__slave__handle.html", [
      [ "callback", "struct__i2c__slave__handle.html#aa1a2ab536368e7b2e8981879f1a3f5ca", null ],
      [ "eventMask", "struct__i2c__slave__handle.html#a769c9b64e73a1ddb99dfbf8cbc244b14", null ],
      [ "isBusy", "struct__i2c__slave__handle.html#af6d7a3aa8f16a2ddacb9cf58977fd67c", null ],
      [ "transfer", "struct__i2c__slave__handle.html#a1dc2ffa11e396aad054370efeb3f2510", null ],
      [ "userData", "struct__i2c__slave__handle.html#a0c21a56aa399763f1969a28fbea35054", null ]
    ] ],
    [ "FSL_I2C_DRIVER_VERSION", "group__i2c__driver.html#ga60fdaec284298758a650ce1122aef231", null ],
    [ "I2C_MASTER_FACK_CONTROL", "group__i2c__driver.html#gace639c1e5e7cdcecb1cca2e52c669029", null ],
    [ "I2C_RETRY_TIMES", "group__i2c__driver.html#gaac288786aaa7d0cb947c91591baf42d7", null ],
    [ "i2c_direction_t", "group__i2c__driver.html#ga8b55706f918048745dd2f6dfd29e8d1b", null ],
    [ "i2c_master_config_t", "group__i2c__driver.html#gadfabd3a0ab169df884871047ab88d824", null ],
    [ "i2c_master_handle_t", "group__i2c__driver.html#ga2fd51f58b8cfd309be985bce4b817fdc", null ],
    [ "i2c_master_transfer_callback_t", "group__i2c__driver.html#gafa39d30cd808d79b774361874f227204", null ],
    [ "i2c_master_transfer_t", "group__i2c__driver.html#ga7edd253f4df51a13bd0a6a0461b9f93a", null ],
    [ "i2c_slave_address_mode_t", "group__i2c__driver.html#ga1f9f1a52a5dec4827c2ac981f0f370c8", null ],
    [ "i2c_slave_config_t", "group__i2c__driver.html#ga274f94465fc90ace978683e1f1e4f4c8", null ],
    [ "i2c_slave_handle_t", "group__i2c__driver.html#ga394e5278816efe62815de9d25be84752", null ],
    [ "i2c_slave_transfer_callback_t", "group__i2c__driver.html#ga2583e3d8ebf8d3de410d8b6263238c35", null ],
    [ "i2c_slave_transfer_event_t", "group__i2c__driver.html#ga0a044d21f51a843c7f559209cf776077", null ],
    [ "i2c_slave_transfer_t", "group__i2c__driver.html#ga14cf3e2a4678d195f68897bef5582ed9", [
      [ "kClearFlags", "group__i2c__driver.html#gga4db40dd1639a06f4bb203cb0eedb2875aa1b63825794cdea9cee35573ef4c35e5", null ],
      [ "kStatus_I2C_Busy", "group__i2c__driver.html#ggad51727b1194e3dbf2d3ac78794389414a49091894b590d7e479605bf113918952", null ],
      [ "kStatus_I2C_Idle", "group__i2c__driver.html#ggad51727b1194e3dbf2d3ac78794389414a628d242f7bc0e3d5949c7f73eafaa508", null ],
      [ "kStatus_I2C_Nak", "group__i2c__driver.html#ggad51727b1194e3dbf2d3ac78794389414a72fd33d0b5263a63766e62f71d16be00", null ],
      [ "kStatus_I2C_ArbitrationLost", "group__i2c__driver.html#ggad51727b1194e3dbf2d3ac78794389414a139b52cc3305ec2c06d0ac94313c221f", null ],
      [ "kStatus_I2C_Timeout", "group__i2c__driver.html#ggad51727b1194e3dbf2d3ac78794389414a81ad7cc198436cabbe91ea55c5288747", null ],
      [ "kStatus_I2C_Addr_Nak", "group__i2c__driver.html#ggad51727b1194e3dbf2d3ac78794389414adf7437bfedcc0d57338ed33f1be5c805", null ]
    ] ],
    [ "_i2c_direction", "group__i2c__driver.html#ga1625497320644ba123718bafdd2c1f39", [
      [ "kI2C_Write", "group__i2c__driver.html#gga1625497320644ba123718bafdd2c1f39a93b476b469c2a4bfed5916b458ae9bb0", null ],
      [ "kI2C_Read", "group__i2c__driver.html#gga1625497320644ba123718bafdd2c1f39a3a7bb24dc8d1c1be8925603eeafe9b30", null ]
    ] ],
    [ "_i2c_flags", "group__i2c__driver.html#ga1f1337bbe9d0b184d9dcee31f9ebade2", [
      [ "kI2C_ReceiveNakFlag", "group__i2c__driver.html#gga1f1337bbe9d0b184d9dcee31f9ebade2a4c3c2928e17ddbfbc1c346c6b9a8dccd", null ],
      [ "kI2C_IntPendingFlag", "group__i2c__driver.html#gga1f1337bbe9d0b184d9dcee31f9ebade2a647d0b1d1f9514ad483bef3ea3926c3b", null ],
      [ "kI2C_TransferDirectionFlag", "group__i2c__driver.html#gga1f1337bbe9d0b184d9dcee31f9ebade2a249f698970909f33b1b96139c05bd969", null ],
      [ "kI2C_RangeAddressMatchFlag", "group__i2c__driver.html#gga1f1337bbe9d0b184d9dcee31f9ebade2aac2c3a26d38c01fba1554eddc0c6b8b5", null ],
      [ "kI2C_ArbitrationLostFlag", "group__i2c__driver.html#gga1f1337bbe9d0b184d9dcee31f9ebade2a39fab5ef04e74d00ef4c4574899ab589", null ],
      [ "kI2C_BusBusyFlag", "group__i2c__driver.html#gga1f1337bbe9d0b184d9dcee31f9ebade2a37fb20742a15a63c1bc2551b997cdcff", null ],
      [ "kI2C_AddressMatchFlag", "group__i2c__driver.html#gga1f1337bbe9d0b184d9dcee31f9ebade2a10cd5797b1b8f809192423486a85d5fe", null ],
      [ "kI2C_TransferCompleteFlag", "group__i2c__driver.html#gga1f1337bbe9d0b184d9dcee31f9ebade2adb8f10365dfc2a3e849f05173fa4d83e", null ]
    ] ],
    [ "_i2c_interrupt_enable", "group__i2c__driver.html#ga87c81dd985dad07dc26cb93125a94ec7", [
      [ "kI2C_GlobalInterruptEnable", "group__i2c__driver.html#gga87c81dd985dad07dc26cb93125a94ec7a5526e27ef6e8317ba928849da890b3b2", null ]
    ] ],
    [ "_i2c_master_transfer_flags", "group__i2c__driver.html#ga87ea07668194cfb46c7c368d2cb42433", [
      [ "kI2C_TransferDefaultFlag", "group__i2c__driver.html#gga87ea07668194cfb46c7c368d2cb42433ae80f7b768b1621e42ac965d3d23de5e2", null ],
      [ "kI2C_TransferNoStartFlag", "group__i2c__driver.html#gga87ea07668194cfb46c7c368d2cb42433a5cb44bf860c0482c0ca0165cf0d31d8a", null ],
      [ "kI2C_TransferRepeatedStartFlag", "group__i2c__driver.html#gga87ea07668194cfb46c7c368d2cb42433aadf112471ef12f194985a093cd9b9721", null ],
      [ "kI2C_TransferNoStopFlag", "group__i2c__driver.html#gga87ea07668194cfb46c7c368d2cb42433afb8aeea71b5b7475e3d3df86220db566", null ]
    ] ],
    [ "_i2c_slave_address_mode", "group__i2c__driver.html#gae1e1e45d8491652a249bf48530911ac7", [
      [ "kI2C_Address7bit", "group__i2c__driver.html#ggae1e1e45d8491652a249bf48530911ac7a3e2c7f1f917939d0d7595224e3700dd4", null ],
      [ "kI2C_RangeMatch", "group__i2c__driver.html#ggae1e1e45d8491652a249bf48530911ac7aa2c312f87691c542c466ff747d21f70a", null ]
    ] ],
    [ "_i2c_slave_transfer_event", "group__i2c__driver.html#ga87e42e170b60f17f657ef3c06a918133", [
      [ "kI2C_SlaveAddressMatchEvent", "group__i2c__driver.html#gga87e42e170b60f17f657ef3c06a918133acb1ae174dd6798a6fd79800a9e39a3c6", null ],
      [ "kI2C_SlaveTransmitEvent", "group__i2c__driver.html#gga87e42e170b60f17f657ef3c06a918133a2f85039a57379838909876a1d509b7aa", null ],
      [ "kI2C_SlaveReceiveEvent", "group__i2c__driver.html#gga87e42e170b60f17f657ef3c06a918133aac8593e808a8137d1b5d3a51640779de", null ],
      [ "kI2C_SlaveTransmitAckEvent", "group__i2c__driver.html#gga87e42e170b60f17f657ef3c06a918133a43c045da27fa60e7a0540c743eca4e79", null ],
      [ "kI2C_SlaveCompletionEvent", "group__i2c__driver.html#gga87e42e170b60f17f657ef3c06a918133a38cca3cd668e73f51b8e574835d419df", null ],
      [ "kI2C_SlaveGenaralcallEvent", "group__i2c__driver.html#gga87e42e170b60f17f657ef3c06a918133a5063c09cccc4d8047aa3d33ff979657b", null ],
      [ "kI2C_SlaveAllEvents", "group__i2c__driver.html#gga87e42e170b60f17f657ef3c06a918133a94b030ebdac378a84961893ae567bfbc", null ]
    ] ],
    [ "I2C_DisableInterrupts", "group__i2c__driver.html#ga9839c1fd49a0f62f42b0adfa74f62853", null ],
    [ "I2C_Enable", "group__i2c__driver.html#gaaaaab1e83cb8c6117046bdb5bcbf9ac6", null ],
    [ "I2C_EnableInterrupts", "group__i2c__driver.html#ga577e094cc4d39b88ec5bbe49bc199c77", null ],
    [ "I2C_GetDataRegAddr", "group__i2c__driver.html#gaf3238d6c3de8e7e8aa7a6c3a8d645769", null ],
    [ "I2C_GetInstance", "group__i2c__driver.html#ga07cc6bd20f700249c335893427bf462a", null ],
    [ "I2C_MasterClearStatusFlags", "group__i2c__driver.html#ga437271ffb955b77df0a5dee9ea80cc63", null ],
    [ "I2C_MasterDeinit", "group__i2c__driver.html#gadcf7122f0a38d4d9da0f052fcb167957", null ],
    [ "I2C_MasterGetDefaultConfig", "group__i2c__driver.html#gad69f2d63ea756dda60749ff6b09f587d", null ],
    [ "I2C_MasterGetStatusFlags", "group__i2c__driver.html#ga721e41295dedc83b424e80e53065e625", null ],
    [ "I2C_MasterInit", "group__i2c__driver.html#ga29f967b3ae8487a36ba2a58deb01ccae", null ],
    [ "I2C_MasterReadBlocking", "group__i2c__driver.html#gaf77526f32a16109abf265dbb022a26ba", null ],
    [ "I2C_MasterRepeatedStart", "group__i2c__driver.html#ga42ff265d25e713326c7d152adf350147", null ],
    [ "I2C_MasterSetBaudRate", "group__i2c__driver.html#gafeab1d5249a8b39c8d9e1a54a85c23f0", null ],
    [ "I2C_MasterStart", "group__i2c__driver.html#ga7664234966e4162e952b6d57efcaa8a6", null ],
    [ "I2C_MasterStop", "group__i2c__driver.html#gaf46a8cc094fc18c6cadea8de71448723", null ],
    [ "I2C_MasterTransferAbort", "group__i2c__driver.html#gabb386e53b2bc64e29b2915bdfac36800", null ],
    [ "I2C_MasterTransferBlocking", "group__i2c__driver.html#ga35cae3a5b4aa50ce8db28e3eb703a027", null ],
    [ "I2C_MasterTransferCreateHandle", "group__i2c__driver.html#ga31e3de02b57801c6896e6045c4a31268", null ],
    [ "I2C_MasterTransferGetCount", "group__i2c__driver.html#ga7d720842e68161d2d52d56d119f20665", null ],
    [ "I2C_MasterTransferHandleIRQ", "group__i2c__driver.html#gad091cd1857d2bd8ce64583739b3a35d5", null ],
    [ "I2C_MasterTransferNonBlocking", "group__i2c__driver.html#gac7f76a04b6f6873c52ad176d58901fe9", null ],
    [ "I2C_MasterWriteBlocking", "group__i2c__driver.html#gac4e1a954243aa852cb4d358e90091efd", null ],
    [ "I2C_SlaveClearStatusFlags", "group__i2c__driver.html#ga545c1a43d8d49c4675de493d45a4063a", null ],
    [ "I2C_SlaveDeinit", "group__i2c__driver.html#gadb50014db29dd75ea2001fb7bc837e77", null ],
    [ "I2C_SlaveGetDefaultConfig", "group__i2c__driver.html#ga7115f80e28b62bbbd3be1a2a918529ba", null ],
    [ "I2C_SlaveGetStatusFlags", "group__i2c__driver.html#gac2794e138d94d9b6a761c074ad0899cd", null ],
    [ "I2C_SlaveInit", "group__i2c__driver.html#gaf2397a88554ea896f212dd262f9bc731", null ],
    [ "I2C_SlaveReadBlocking", "group__i2c__driver.html#ga89baaf8bc82b19c837032665e89a1707", null ],
    [ "I2C_SlaveTransferAbort", "group__i2c__driver.html#ga5ae9f5e6e854d14d33ba3029128bd6f0", null ],
    [ "I2C_SlaveTransferCreateHandle", "group__i2c__driver.html#gac9b90f575d92ff8ad3cc350a5c8ad1b9", null ],
    [ "I2C_SlaveTransferGetCount", "group__i2c__driver.html#gabf38a498bd3527ea0b7cc947ae71e17e", null ],
    [ "I2C_SlaveTransferHandleIRQ", "group__i2c__driver.html#ga097c0b20503381de44cccba311dcf236", null ],
    [ "I2C_SlaveTransferNonBlocking", "group__i2c__driver.html#gaa387d81ea08a9d4b39cfeb3dfec505f7", null ],
    [ "I2C_SlaveWriteBlocking", "group__i2c__driver.html#ga280404c6b1ede542f2c1c20bd382869c", null ]
];