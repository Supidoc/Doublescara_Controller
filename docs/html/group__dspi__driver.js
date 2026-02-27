var group__dspi__driver =
[
    [ "_dspi_command_data_config", "struct__dspi__command__data__config.html", [
      [ "clearTransferCount", "struct__dspi__command__data__config.html#a7acf5e6ccb1703a662fe5ed85eb007a7", null ],
      [ "isEndOfQueue", "struct__dspi__command__data__config.html#a00abc6ff2739b315c3fec567b1f22220", null ],
      [ "isPcsContinuous", "struct__dspi__command__data__config.html#ab0b933370779614acebf4d6ae0bd12ea", null ],
      [ "whichCtar", "struct__dspi__command__data__config.html#a1d676067833f1c64ed1c5bbcd29dcae3", null ],
      [ "whichPcs", "struct__dspi__command__data__config.html#a91c821a7c685fbd300d7583250de2bb2", null ]
    ] ],
    [ "_dspi_master_ctar_config", "struct__dspi__master__ctar__config.html", [
      [ "baudRate", "struct__dspi__master__ctar__config.html#a7a6dec649dade596f2757e1906ab48a1", null ],
      [ "betweenTransferDelayInNanoSec", "struct__dspi__master__ctar__config.html#a462b27cfa47a557c8bf08a8f40988c88", null ],
      [ "bitsPerFrame", "struct__dspi__master__ctar__config.html#a385b7266d1d57cb7d15d576317f2e4a2", null ],
      [ "cpha", "struct__dspi__master__ctar__config.html#a67c82b257b6c4e4d823c0179f09faf1c", null ],
      [ "cpol", "struct__dspi__master__ctar__config.html#a8416c683ca9b05396a16e7bfaefa8f29", null ],
      [ "direction", "struct__dspi__master__ctar__config.html#a8cdc43cfef23bd2ad8640718c40a971d", null ],
      [ "lastSckToPcsDelayInNanoSec", "struct__dspi__master__ctar__config.html#a6b6dd7bfad9bffc90760074b5fc1eeb0", null ],
      [ "pcsToSckDelayInNanoSec", "struct__dspi__master__ctar__config.html#a0f47f98c7c42956cd39ccc004b0e2dd8", null ]
    ] ],
    [ "_dspi_master_config", "struct__dspi__master__config.html", [
      [ "ctarConfig", "struct__dspi__master__config.html#ac531edaac8b2eedd837ea80c498db0e1", null ],
      [ "enableContinuousSCK", "struct__dspi__master__config.html#abe5dceb7c96289e8c642d9467a486abf", null ],
      [ "enableModifiedTimingFormat", "struct__dspi__master__config.html#a581f608743a8cd48e2b3190cc71f8c67", null ],
      [ "enableRxFifoOverWrite", "struct__dspi__master__config.html#ae5099dfdbdb3a30141e9f2d0892d4a3f", null ],
      [ "pcsActiveHighOrLow", "struct__dspi__master__config.html#ab49ecacb84021530a140b136ed8ea167", null ],
      [ "samplePoint", "struct__dspi__master__config.html#a51c98d2b4782bcecd6dd2492490c0c88", null ],
      [ "whichCtar", "struct__dspi__master__config.html#a2a90fd75419695f73e55024d0e380968", null ],
      [ "whichPcs", "struct__dspi__master__config.html#ae28020406cfbc2d6b14dae6012b72c0e", null ]
    ] ],
    [ "_dspi_slave_ctar_config", "struct__dspi__slave__ctar__config.html", [
      [ "bitsPerFrame", "struct__dspi__slave__ctar__config.html#aa1072b1d172632b987b9d5089aa304ae", null ],
      [ "cpha", "struct__dspi__slave__ctar__config.html#a6a3af1df66c2c64a9c6290ab4c1b8153", null ],
      [ "cpol", "struct__dspi__slave__ctar__config.html#a5cbe1b143502dfdd3cb01a95f2b9c307", null ]
    ] ],
    [ "_dspi_slave_config", "struct__dspi__slave__config.html", [
      [ "ctarConfig", "struct__dspi__slave__config.html#a862983aef83e5489acadbbb07e798ce8", null ],
      [ "enableContinuousSCK", "struct__dspi__slave__config.html#a8b44c3830768c44f6d8335136be0f32e", null ],
      [ "enableModifiedTimingFormat", "struct__dspi__slave__config.html#a47e13dd65da7731cef46406cbdd778bc", null ],
      [ "enableRxFifoOverWrite", "struct__dspi__slave__config.html#a8c38b818e168db73ba8a990caf0f1312", null ],
      [ "samplePoint", "struct__dspi__slave__config.html#a319156c9d26e602538073360f0cba170", null ],
      [ "whichCtar", "struct__dspi__slave__config.html#ab14c7d894a1faab557db16efcebb3fd0", null ]
    ] ],
    [ "_dspi_transfer", "struct__dspi__transfer.html", [
      [ "configFlags", "struct__dspi__transfer.html#abd27395d09865e2a7721680bbd4f7c9f", null ],
      [ "dataSize", "struct__dspi__transfer.html#a76dfcfab365d75f91760e0f0ef488411", null ],
      [ "rxData", "struct__dspi__transfer.html#a68f936740e8d07eb57a7dd2a172d1218", null ],
      [ "txData", "struct__dspi__transfer.html#a6e7f17fbe756728256cc3639cec8700b", null ]
    ] ],
    [ "_dspi_half_duplex_transfer", "struct__dspi__half__duplex__transfer.html", [
      [ "configFlags", "struct__dspi__half__duplex__transfer.html#af9f9a33e102149b0fe0efb85ef5cc4d2", null ],
      [ "isPcsAssertInTransfer", "struct__dspi__half__duplex__transfer.html#ae4580687573f09998707f468a2f285bd", null ],
      [ "isTransmitFirst", "struct__dspi__half__duplex__transfer.html#aad15076035f5e87aeea462205f63a576", null ],
      [ "rxData", "struct__dspi__half__duplex__transfer.html#acdca4631bcf16956881a3dae227f2ddf", null ],
      [ "rxDataSize", "struct__dspi__half__duplex__transfer.html#aea68b51418541eb7da1a8da080a3d886", null ],
      [ "txData", "struct__dspi__half__duplex__transfer.html#a02f15d8219b37bca607b18d3e86c0d78", null ],
      [ "txDataSize", "struct__dspi__half__duplex__transfer.html#a4fcf0cb2676f6048382d97faef537977", null ]
    ] ],
    [ "_dspi_master_handle", "struct__dspi__master__handle.html", [
      [ "bitsPerFrame", "struct__dspi__master__handle.html#ab8bd1f2570a90f8582361561dd166cc9", null ],
      [ "callback", "struct__dspi__master__handle.html#a0720a6418252e5aa3491836b2bac21e0", null ],
      [ "command", "struct__dspi__master__handle.html#ad1e93e710eea449346b19fe5482a07fb", null ],
      [ "fifoSize", "struct__dspi__master__handle.html#a0670bcfac862dfd5ec4a48feb393c5f6", null ],
      [ "isPcsActiveAfterTransfer", "struct__dspi__master__handle.html#ae32b0aa49ac7cbd0930a043c9f63f247", null ],
      [ "isThereExtraByte", "struct__dspi__master__handle.html#a97ce98a93978392a552736ac2a13d45d", null ],
      [ "lastCommand", "struct__dspi__master__handle.html#a76f50ee84dc6b4c21a8267d77ce18a34", null ],
      [ "remainingReceiveByteCount", "struct__dspi__master__handle.html#a0f5689554fd1f060b85574a17c1e8d10", null ],
      [ "remainingSendByteCount", "struct__dspi__master__handle.html#a13885d3cc810978301c189a5bb538382", null ],
      [ "rxData", "struct__dspi__master__handle.html#a463fc26daab6e441a228cc24114341b4", null ],
      [ "state", "struct__dspi__master__handle.html#a3f069f1061f77e0a2e3ef0003fea0176", null ],
      [ "totalByteCount", "struct__dspi__master__handle.html#af64f29648e449a4049f473eb5869d00a", null ],
      [ "txData", "struct__dspi__master__handle.html#a2e5c2974db7ae0b45b449240987c8018", null ],
      [ "userData", "struct__dspi__master__handle.html#afa2a59612f2b99e134027a4cda2dad26", null ]
    ] ],
    [ "_dspi_slave_handle", "struct__dspi__slave__handle.html", [
      [ "bitsPerFrame", "struct__dspi__slave__handle.html#a58a326a590d24036fdfcc713d7bb0a82", null ],
      [ "callback", "struct__dspi__slave__handle.html#ae59eddf710ce24c8046bfc56b75baa34", null ],
      [ "errorCount", "struct__dspi__slave__handle.html#a8402d098f781e53d885f4beeb75bb4bc", null ],
      [ "isThereExtraByte", "struct__dspi__slave__handle.html#a13906acad5ef5d6685f9a08e13f76824", null ],
      [ "remainingReceiveByteCount", "struct__dspi__slave__handle.html#a4767de5e5c0cfcc17692b15032643c1d", null ],
      [ "remainingSendByteCount", "struct__dspi__slave__handle.html#aa48edca370f47db153df82be8973b613", null ],
      [ "rxData", "struct__dspi__slave__handle.html#a1a9358da2c1255795a315e4e3450e020", null ],
      [ "state", "struct__dspi__slave__handle.html#a0830c41d7b765067e427ae7c1a7683d2", null ],
      [ "totalByteCount", "struct__dspi__slave__handle.html#a8b8c327e317d3d4ddcfd4346c11163d1", null ],
      [ "txData", "struct__dspi__slave__handle.html#a45bb14f073972feb6e9a86b98d8009e9", null ],
      [ "userData", "struct__dspi__slave__handle.html#a5b455d8a0a435d8c160a2f55f709535b", null ]
    ] ],
    [ "DSPI_DUMMY_DATA", "group__dspi__driver.html#gae3af6268fdc846d553d23c82fd8c8668", null ],
    [ "DSPI_MASTER_CTAR_MASK", "group__dspi__driver.html#ga15e4a446e06e9964a77d20c00b7d3397", null ],
    [ "DSPI_MASTER_CTAR_SHIFT", "group__dspi__driver.html#gac4a272cf2546e7d51aa45d76b10dc174", null ],
    [ "DSPI_MASTER_PCS_MASK", "group__dspi__driver.html#ga213b335dc840f0c1637ca37b0cf3f4f4", null ],
    [ "DSPI_MASTER_PCS_SHIFT", "group__dspi__driver.html#ga082f0377ee21656002efecaeaf77034c", null ],
    [ "DSPI_SLAVE_CTAR_MASK", "group__dspi__driver.html#gaf41a736795e34277fdb594a75754ab69", null ],
    [ "DSPI_SLAVE_CTAR_SHIFT", "group__dspi__driver.html#ga5d0819629aecb560192865e1850bff07", null ],
    [ "FSL_DSPI_DRIVER_VERSION", "group__dspi__driver.html#ga9ed59b934c560c5d88000b17b8171a01", null ],
    [ "dspi_clock_phase_t", "group__dspi__driver.html#gacab9a1282f3fe347704991d021242bc5", null ],
    [ "dspi_clock_polarity_t", "group__dspi__driver.html#ga1b58260ac34e4dd32642ece9f76bd624", null ],
    [ "dspi_command_data_config_t", "group__dspi__driver.html#gaafee4e516883fc4294907f6a4dc43ada", null ],
    [ "dspi_ctar_selection_t", "group__dspi__driver.html#ga971efc5f40742e64a1a3396e00c33a1f", null ],
    [ "dspi_delay_type_t", "group__dspi__driver.html#ga4fecd586e93f31e03c6ec63097859a60", null ],
    [ "dspi_half_duplex_transfer_t", "group__dspi__driver.html#ga2fdcc5b94bd24b22fa7726a6f728b1a3", null ],
    [ "dspi_master_config_t", "group__dspi__driver.html#gae28f1221ec3ce6efd7054c1e0b63b272", null ],
    [ "dspi_master_ctar_config_t", "group__dspi__driver.html#gaa563bdc990e2e201fe3f1fedb9e804c7", null ],
    [ "dspi_master_handle_t", "group__dspi__driver.html#gaf099ad07783e86fb0f7806c4e7d62a92", null ],
    [ "dspi_master_sample_point_t", "group__dspi__driver.html#gac7f694e339b1d84a29c35e9e033b0534", null ],
    [ "dspi_master_slave_mode_t", "group__dspi__driver.html#gae5e998a7409d1f79afc81f2c03a17903", null ],
    [ "dspi_master_transfer_callback_t", "group__dspi__driver.html#ga67d91a817bd68468037b7886ea710ffa", null ],
    [ "dspi_pcs_polarity_config_t", "group__dspi__driver.html#ga936908d6b73abdb743dbc8de43dc5b4f", null ],
    [ "dspi_shift_direction_t", "group__dspi__driver.html#ga5a070bf787f65b7ca240e0c3f85adac9", null ],
    [ "dspi_slave_config_t", "group__dspi__driver.html#ga21df497b86f74cff62e5d85754973155", null ],
    [ "dspi_slave_ctar_config_t", "group__dspi__driver.html#gaa66f1b321f738852ff90c99ecf8b5be9", null ],
    [ "dspi_slave_handle_t", "group__dspi__driver.html#ga732be23fdf8a842c3e05db1f2f6d8fb9", null ],
    [ "dspi_slave_transfer_callback_t", "group__dspi__driver.html#gaf928cd95fbb7eb5d5f306c9a4d4afce4", null ],
    [ "dspi_transfer_t", "group__dspi__driver.html#gabf2f85110e09409e49b42d122f94e592", null ],
    [ "dspi_which_pcs_t", "group__dspi__driver.html#gaea0bd6fc00a637ff53e6e2f482b28ca0", [
      [ "kStatus_DSPI_Busy", "group__dspi__driver.html#gga186fcac0c06a158b9d0166ea0f2a6b62ad5ecc8346da4119b8609c6bcb4c57e40", null ],
      [ "kStatus_DSPI_Error", "group__dspi__driver.html#gga186fcac0c06a158b9d0166ea0f2a6b62aaae2ba17a6a622142816b0ffec7b9f7a", null ],
      [ "kStatus_DSPI_Idle", "group__dspi__driver.html#gga186fcac0c06a158b9d0166ea0f2a6b62a638dc0d050e7660225a46cc7cd6e38c7", null ],
      [ "kStatus_DSPI_OutOfRange", "group__dspi__driver.html#gga186fcac0c06a158b9d0166ea0f2a6b62ac1713712f0410e28da008d714734a6bd", null ]
    ] ],
    [ "_dspi_clock_phase", "group__dspi__driver.html#ga648d70d13ec12505a80c423841f53510", [
      [ "kDSPI_ClockPhaseFirstEdge", "group__dspi__driver.html#gga648d70d13ec12505a80c423841f53510a996e921abbf325ee9978a42681aee0d5", null ],
      [ "kDSPI_ClockPhaseSecondEdge", "group__dspi__driver.html#gga648d70d13ec12505a80c423841f53510a43ee643e847b3118e38da0a9811d97f9", null ]
    ] ],
    [ "_dspi_clock_polarity", "group__dspi__driver.html#ga8324f34ea4af085c11052288fc94983e", [
      [ "kDSPI_ClockPolarityActiveHigh", "group__dspi__driver.html#gga8324f34ea4af085c11052288fc94983eab5279f36f0c6b1617aa937824806d71d", null ],
      [ "kDSPI_ClockPolarityActiveLow", "group__dspi__driver.html#gga8324f34ea4af085c11052288fc94983eabcde58b8834e5cd1181b8b98aa4a10ef", null ]
    ] ],
    [ "_dspi_ctar_selection", "group__dspi__driver.html#ga78060c4222b1affc8e41f937909ee999", [
      [ "kDSPI_Ctar0", "group__dspi__driver.html#gga78060c4222b1affc8e41f937909ee999adb2a4c8c9b722c6a1b8cbb03b17a6519", null ],
      [ "kDSPI_Ctar1", "group__dspi__driver.html#gga78060c4222b1affc8e41f937909ee999ad6db3f5779fd74fdfa9bda2375573227", null ],
      [ "kDSPI_Ctar2", "group__dspi__driver.html#gga78060c4222b1affc8e41f937909ee999a406d09f42f5e009617a40f4c30cc10d9", null ],
      [ "kDSPI_Ctar3", "group__dspi__driver.html#gga78060c4222b1affc8e41f937909ee999af1df973bc8d89efbfb8d7bff51af0265", null ],
      [ "kDSPI_Ctar4", "group__dspi__driver.html#gga78060c4222b1affc8e41f937909ee999a13960000166ae1cc18b19f5c4c9405ff", null ],
      [ "kDSPI_Ctar5", "group__dspi__driver.html#gga78060c4222b1affc8e41f937909ee999ad0b231829a94051ce913cd367135c1f2", null ],
      [ "kDSPI_Ctar6", "group__dspi__driver.html#gga78060c4222b1affc8e41f937909ee999aa7bb6aaabeb65811e58af0460c38e373", null ],
      [ "kDSPI_Ctar7", "group__dspi__driver.html#gga78060c4222b1affc8e41f937909ee999a6ae1a9c5243a507f36c3db1ef14c216e", null ]
    ] ],
    [ "_dspi_delay_type", "group__dspi__driver.html#ga86dbb9c380b74c6654a44a73e90573f9", [
      [ "kDSPI_PcsToSck", "group__dspi__driver.html#gga86dbb9c380b74c6654a44a73e90573f9a71185ae0d4d9dd61acbc69bce93f33f5", null ],
      [ "kDSPI_LastSckToPcs", "group__dspi__driver.html#gga86dbb9c380b74c6654a44a73e90573f9aa2ce775b9575a3870ce82b8444b9d56c", null ],
      [ "kDSPI_BetweenTransfer", "group__dspi__driver.html#gga86dbb9c380b74c6654a44a73e90573f9a83ed3f05b8a61f94c0da066c1ded7a1e", null ]
    ] ],
    [ "_dspi_dma_enable", "group__dspi__driver.html#gae3359796dc0680797b1f74b83fc0c0d9", [
      [ "kDSPI_TxDmaEnable", "group__dspi__driver.html#ggae3359796dc0680797b1f74b83fc0c0d9ae772dc49e5a28df00b817f9c6dab0749", null ],
      [ "kDSPI_RxDmaEnable", "group__dspi__driver.html#ggae3359796dc0680797b1f74b83fc0c0d9a15ec9c9897199d53a1b354ccce6d0445", null ]
    ] ],
    [ "_dspi_flags", "group__dspi__driver.html#ga2bfefaf6ba65ba464e764d1c918c904f", [
      [ "kDSPI_TxCompleteFlag", "group__dspi__driver.html#gga2bfefaf6ba65ba464e764d1c918c904faffc8e8711d9083470cddb0db647b75b0", null ],
      [ "kDSPI_EndOfQueueFlag", "group__dspi__driver.html#gga2bfefaf6ba65ba464e764d1c918c904fae91c7a5cc2a90fa051c89f13bbb6d8ed", null ],
      [ "kDSPI_TxFifoUnderflowFlag", "group__dspi__driver.html#gga2bfefaf6ba65ba464e764d1c918c904fae36215137d8ce7cf215349199db877b7", null ],
      [ "kDSPI_TxFifoFillRequestFlag", "group__dspi__driver.html#gga2bfefaf6ba65ba464e764d1c918c904fae9704d53b57758969f8ea5ea6c86f7f0", null ],
      [ "kDSPI_RxFifoOverflowFlag", "group__dspi__driver.html#gga2bfefaf6ba65ba464e764d1c918c904fa30f039adca01f89dbbd02f70dff725ee", null ],
      [ "kDSPI_RxFifoDrainRequestFlag", "group__dspi__driver.html#gga2bfefaf6ba65ba464e764d1c918c904fa092b7f39357ce8cb82ec825e93536605", null ],
      [ "kDSPI_TxAndRxStatusFlag", "group__dspi__driver.html#gga2bfefaf6ba65ba464e764d1c918c904fa58771b3977aef221dab6a67a6739f8d6", null ],
      [ "kDSPI_AllStatusFlag", "group__dspi__driver.html#gga2bfefaf6ba65ba464e764d1c918c904fa4a742818251256d8fc35ab63a6af9c9e", null ]
    ] ],
    [ "_dspi_interrupt_enable", "group__dspi__driver.html#gaeb57298690a2f1a09d94d696c893c4b2", [
      [ "kDSPI_TxCompleteInterruptEnable", "group__dspi__driver.html#ggaeb57298690a2f1a09d94d696c893c4b2ab2b1ba228fd75de23a2de7e56c1ee438", null ],
      [ "kDSPI_EndOfQueueInterruptEnable", "group__dspi__driver.html#ggaeb57298690a2f1a09d94d696c893c4b2a069483b28469fcbfa5890b04cd6439b3", null ],
      [ "kDSPI_TxFifoUnderflowInterruptEnable", "group__dspi__driver.html#ggaeb57298690a2f1a09d94d696c893c4b2aa430e623e0bb240752381eaddda1a973", null ],
      [ "kDSPI_TxFifoFillRequestInterruptEnable", "group__dspi__driver.html#ggaeb57298690a2f1a09d94d696c893c4b2ada57830661d523d12e49892060fde201", null ],
      [ "kDSPI_RxFifoOverflowInterruptEnable", "group__dspi__driver.html#ggaeb57298690a2f1a09d94d696c893c4b2a190746a0aeaa61db32c6c1a7b850d0ee", null ],
      [ "kDSPI_RxFifoDrainRequestInterruptEnable", "group__dspi__driver.html#ggaeb57298690a2f1a09d94d696c893c4b2aa7d99e6ac31bd6c7d835d89f36cec1a6", null ],
      [ "kDSPI_AllInterruptEnable", "group__dspi__driver.html#ggaeb57298690a2f1a09d94d696c893c4b2a530d972d6cd16ab6e929d7ddaaf09b30", null ]
    ] ],
    [ "_dspi_master_sample_point", "group__dspi__driver.html#ga02a53597bff1469f0365b74ad7a20237", [
      [ "kDSPI_SckToSin0Clock", "group__dspi__driver.html#gga02a53597bff1469f0365b74ad7a20237abbcf84bafbd94a63a9600647162b8d86", null ],
      [ "kDSPI_SckToSin1Clock", "group__dspi__driver.html#gga02a53597bff1469f0365b74ad7a20237a61e5f5d7122c849c737513ae7c5c4c50", null ],
      [ "kDSPI_SckToSin2Clock", "group__dspi__driver.html#gga02a53597bff1469f0365b74ad7a20237a305d68c9446ca0866da7a2ace743ae4d", null ]
    ] ],
    [ "_dspi_master_slave_mode", "group__dspi__driver.html#ga22d5d3420ce510463f61e41cbe1d1410", [
      [ "kDSPI_Master", "group__dspi__driver.html#gga22d5d3420ce510463f61e41cbe1d1410a8330c6ad827da3c783df5805244fa7d9", null ],
      [ "kDSPI_Slave", "group__dspi__driver.html#gga22d5d3420ce510463f61e41cbe1d1410a2e075745386fd71bee2535606f29dd87", null ]
    ] ],
    [ "_dspi_pcs_polarity", "group__dspi__driver.html#gad23a66cefb04826de83504ad485f19a9", [
      [ "kDSPI_Pcs0ActiveLow", "group__dspi__driver.html#ggad23a66cefb04826de83504ad485f19a9ac731b21eefcc16342d2c606a12a00547", null ],
      [ "kDSPI_Pcs1ActiveLow", "group__dspi__driver.html#ggad23a66cefb04826de83504ad485f19a9aa6ee5dca40cbe9bf03623cf986adbadd", null ],
      [ "kDSPI_Pcs2ActiveLow", "group__dspi__driver.html#ggad23a66cefb04826de83504ad485f19a9a6fd76d22cb6c8f943ae397bb91ba68f4", null ],
      [ "kDSPI_Pcs3ActiveLow", "group__dspi__driver.html#ggad23a66cefb04826de83504ad485f19a9a57e33d7e4195864f89db11d2f5e6cc4b", null ],
      [ "kDSPI_Pcs4ActiveLow", "group__dspi__driver.html#ggad23a66cefb04826de83504ad485f19a9a15c201d8e7bd0bab1dd7117b73a111ec", null ],
      [ "kDSPI_Pcs5ActiveLow", "group__dspi__driver.html#ggad23a66cefb04826de83504ad485f19a9a88e1e00a5a7755561358f004a5a1b1d4", null ],
      [ "kDSPI_PcsAllActiveLow", "group__dspi__driver.html#ggad23a66cefb04826de83504ad485f19a9adb2bef5058b4bf00533cc89f1928e2d1", null ]
    ] ],
    [ "_dspi_pcs_polarity_config", "group__dspi__driver.html#ga1b2a1103a54ad51c35f2bdaa52ac7363", [
      [ "kDSPI_PcsActiveHigh", "group__dspi__driver.html#gga1b2a1103a54ad51c35f2bdaa52ac7363a79a6807edd30a1230477ab26068060fd", null ],
      [ "kDSPI_PcsActiveLow", "group__dspi__driver.html#gga1b2a1103a54ad51c35f2bdaa52ac7363aa678a5937bbb9975e3c014592c3d542c", null ]
    ] ],
    [ "_dspi_shift_direction", "group__dspi__driver.html#gaf1134e11fc318e82a6c15882fdab2760", [
      [ "kDSPI_MsbFirst", "group__dspi__driver.html#ggaf1134e11fc318e82a6c15882fdab2760a8885a916a15d0b97ffd0f28d81242f6f", null ],
      [ "kDSPI_LsbFirst", "group__dspi__driver.html#ggaf1134e11fc318e82a6c15882fdab2760a76701314fa7dbd70e4011feb326b9050", null ]
    ] ],
    [ "_dspi_transfer_config_flag_for_master", "group__dspi__driver.html#gac74dfe19c844271a393314a4fd13792f", [
      [ "kDSPI_MasterCtar0", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792faf7ddf42278af30a1b81f10c4058ecddd", null ],
      [ "kDSPI_MasterCtar1", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792fa57508605f5d5fb0a2fb7eddfcdb89f12", null ],
      [ "kDSPI_MasterCtar2", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792fa6cf50df8fd75f5be1347efcaec8a68f4", null ],
      [ "kDSPI_MasterCtar3", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792fa70471fdf900dba881f4e742d303d307c", null ],
      [ "kDSPI_MasterCtar4", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792faad989e96bfed1f2fbb0fcc3adb99d04b", null ],
      [ "kDSPI_MasterCtar5", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792fa5c3dbe0ddb8e9f3f67496592ef3ec902", null ],
      [ "kDSPI_MasterCtar6", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792fa5e898da1cd4e093f048f947bc751b7fa", null ],
      [ "kDSPI_MasterCtar7", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792fa90cf553b9933d1e3d692469e0fa5ddc3", null ],
      [ "kDSPI_MasterPcs0", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792fad51bd34d51062d900b07801e0fd193cc", null ],
      [ "kDSPI_MasterPcs1", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792fad07c95fafd30869cb6110d4ea3ed7ca1", null ],
      [ "kDSPI_MasterPcs2", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792fa116fef8c0a72727a80e72e1d1d0d0ffc", null ],
      [ "kDSPI_MasterPcs3", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792fa2266cc2ddbf05da3164fa6ad680facd9", null ],
      [ "kDSPI_MasterPcs4", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792fa3b32f4a57a5aaaaf0064d7ec1373a154", null ],
      [ "kDSPI_MasterPcs5", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792fa8047faff72926a57c0659f4147787353", null ],
      [ "kDSPI_MasterPcsContinuous", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792fa8309b1b52bbaa930bbcc3e2407f1a6f5", null ],
      [ "kDSPI_MasterActiveAfterTransfer", "group__dspi__driver.html#ggac74dfe19c844271a393314a4fd13792fa458df11cc493759474f31873cfa8d4c1", null ]
    ] ],
    [ "_dspi_transfer_config_flag_for_slave", "group__dspi__driver.html#ga5070a73633ee72428adda72058f7fb5f", [
      [ "kDSPI_SlaveCtar0", "group__dspi__driver.html#gga5070a73633ee72428adda72058f7fb5fa6e63c217f9b392f78fb96ee039c991c8", null ]
    ] ],
    [ "_dspi_transfer_state", "group__dspi__driver.html#ga97c65523863f89cddbf06691c678a7f9", [
      [ "kDSPI_Idle", "group__dspi__driver.html#gga97c65523863f89cddbf06691c678a7f9ae739fb0dabff3a7cb72c39eef943a373", null ],
      [ "kDSPI_Busy", "group__dspi__driver.html#gga97c65523863f89cddbf06691c678a7f9a4b636d65ab83d136e81ed31e30de4429", null ],
      [ "kDSPI_Error", "group__dspi__driver.html#gga97c65523863f89cddbf06691c678a7f9a6d94f11a50f542371683efe9ea22efb9", null ]
    ] ],
    [ "_dspi_which_pcs_config", "group__dspi__driver.html#ga738d1c33c04047440e86e29fabbbba94", [
      [ "kDSPI_Pcs0", "group__dspi__driver.html#gga738d1c33c04047440e86e29fabbbba94a5c6297be9586ee874fa1a84a16d810b7", null ],
      [ "kDSPI_Pcs1", "group__dspi__driver.html#gga738d1c33c04047440e86e29fabbbba94a62d3c43292cebeed478a36bff2cd033a", null ],
      [ "kDSPI_Pcs2", "group__dspi__driver.html#gga738d1c33c04047440e86e29fabbbba94a625c90d5151e8458be6f89ace68f2fe2", null ],
      [ "kDSPI_Pcs3", "group__dspi__driver.html#gga738d1c33c04047440e86e29fabbbba94a7fae848c0f775a86562b90ecfd171cc8", null ],
      [ "kDSPI_Pcs4", "group__dspi__driver.html#gga738d1c33c04047440e86e29fabbbba94a0fd968cdbfd2e088987e309f49cb20f2", null ],
      [ "kDSPI_Pcs5", "group__dspi__driver.html#gga738d1c33c04047440e86e29fabbbba94a67653d39cbd675c9141bb014d4576a0b", null ]
    ] ],
    [ "DSPI_ClearStatusFlags", "group__dspi__driver.html#ga11454768ad4c96b65b298cccf1f0401c", null ],
    [ "DSPI_Deinit", "group__dspi__driver.html#gaa669bb8f6438b1d4f7ec38ba180653fa", null ],
    [ "DSPI_DisableDMA", "group__dspi__driver.html#ga543b12952cb5ac404ebbdaa572628c8e", null ],
    [ "DSPI_DisableInterrupts", "group__dspi__driver.html#gabf5c4ec1216387b8c476853e45a9bfeb", null ],
    [ "DSPI_Enable", "group__dspi__driver.html#ga38a2ee1ed351246ebbdc4b242b835164", null ],
    [ "DSPI_EnableDMA", "group__dspi__driver.html#ga313d41fd54ca75781bb7596b319d4849", null ],
    [ "DSPI_EnableInterrupts", "group__dspi__driver.html#ga9b9e4c8ae54ea108952c80940e11b3a8", null ],
    [ "DSPI_FlushFifo", "group__dspi__driver.html#ga3cbb532b5bd6981f5cc0115f49a9ee9a", null ],
    [ "DSPI_GetDefaultDataCommandConfig", "group__dspi__driver.html#gad9f3df616e7284696af57cce8f49899e", null ],
    [ "DSPI_GetDummyDataInstance", "group__dspi__driver.html#ga055ff1f9fa7139ddc77f9457af96a9c3", null ],
    [ "DSPI_GetInstance", "group__dspi__driver.html#ga55a8d287feaf774cb3a7b644171e0f2f", null ],
    [ "DSPI_GetRxRegisterAddress", "group__dspi__driver.html#ga0d2bcb0a744852ab2701466a7fd974f6", null ],
    [ "DSPI_GetStatusFlags", "group__dspi__driver.html#ga11005216bf792c91894d9e670b0323f8", null ],
    [ "DSPI_IsMaster", "group__dspi__driver.html#gae606c91960692b493d17d067c38d67b3", null ],
    [ "DSPI_MasterGetDefaultConfig", "group__dspi__driver.html#ga0061c90bc787dc1faffde79cb256e8a4", null ],
    [ "DSPI_MasterGetFormattedCommand", "group__dspi__driver.html#ga4068b27da40c419a700badf2070fc5e4", null ],
    [ "DSPI_MasterGetTxRegisterAddress", "group__dspi__driver.html#gad3e8a8107cfda29dbae45fc5166d63f3", null ],
    [ "DSPI_MasterHalfDuplexTransferBlocking", "group__dspi__driver.html#ga4e6b7b248a7cc158a17876b651f4c869", null ],
    [ "DSPI_MasterHalfDuplexTransferNonBlocking", "group__dspi__driver.html#gaf6f6e6e90ff9819e53660a154e0a2aa1", null ],
    [ "DSPI_MasterInit", "group__dspi__driver.html#gaadf23f732f4c1b61d6634bd17b1a36d7", null ],
    [ "DSPI_MasterSetBaudRate", "group__dspi__driver.html#gac76cf793dd837dd0b502770913058592", null ],
    [ "DSPI_MasterSetDelayScaler", "group__dspi__driver.html#ga56d5b87114e56507c0ec2d631ffefaa2", null ],
    [ "DSPI_MasterSetDelayTimes", "group__dspi__driver.html#gac60f64fd410404ebab553ee878b464c2", null ],
    [ "DSPI_MasterTransferAbort", "group__dspi__driver.html#ga80633e998c10cb83685d6c64ecd33a55", null ],
    [ "DSPI_MasterTransferBlocking", "group__dspi__driver.html#gab2d0aa3acb2acc3cc5413314d758628b", null ],
    [ "DSPI_MasterTransferCreateHandle", "group__dspi__driver.html#ga63e04b92d99d795cf84df62379765a91", null ],
    [ "DSPI_MasterTransferGetCount", "group__dspi__driver.html#gadaf98a7213c03f10d5820d363e827a73", null ],
    [ "DSPI_MasterTransferHandleIRQ", "group__dspi__driver.html#ga195eed1bfdc0d21e7adb76a5d6d247dc", null ],
    [ "DSPI_MasterTransferNonBlocking", "group__dspi__driver.html#gad3dc7b85b448ce6e16e227d7bf3769d6", null ],
    [ "DSPI_MasterWriteCommandDataBlocking", "group__dspi__driver.html#ga0718581088422b572cb4494f26aad1f9", null ],
    [ "DSPI_MasterWriteData", "group__dspi__driver.html#gabe0d615b273c4cb0eaf26d9679b73ad6", null ],
    [ "DSPI_MasterWriteDataBlocking", "group__dspi__driver.html#ga70a0f7d7fe2fbce7993bbcc8c427b2b0", null ],
    [ "DSPI_ReadData", "group__dspi__driver.html#gaee93673062a6fb105dcf1e0541dd8b52", null ],
    [ "DSPI_SetAllPcsPolarity", "group__dspi__driver.html#ga1c42d5efc75982041f4a66f4f1fc71a4", null ],
    [ "DSPI_SetDummyData", "group__dspi__driver.html#ga6601ce5a69dfa993874baa4f2539252d", null ],
    [ "DSPI_SetFifoEnable", "group__dspi__driver.html#gad9112153c575eeeb6af747d9e6396514", null ],
    [ "DSPI_SetMasterSlaveMode", "group__dspi__driver.html#gac3e11f3876e81d7636a77fb268c2365a", null ],
    [ "DSPI_SlaveGetDefaultConfig", "group__dspi__driver.html#gad85a8d4e7bd2747103691a63ef9a67e1", null ],
    [ "DSPI_SlaveGetTxRegisterAddress", "group__dspi__driver.html#ga8912754715dfadde5473a419f7b8ff93", null ],
    [ "DSPI_SlaveInit", "group__dspi__driver.html#gacf6cecb6b73f02eaa448634a8d705851", null ],
    [ "DSPI_SlaveTransferAbort", "group__dspi__driver.html#ga7e1be1f74fd8d372ce1af52c960d1361", null ],
    [ "DSPI_SlaveTransferCreateHandle", "group__dspi__driver.html#gadc23691aa2c06ae9076a5f0b16f33a8c", null ],
    [ "DSPI_SlaveTransferGetCount", "group__dspi__driver.html#ga4134bb536420951e8ecbe8edb987d199", null ],
    [ "DSPI_SlaveTransferHandleIRQ", "group__dspi__driver.html#gade8288c503cc6c7af542cdc86947ecd3", null ],
    [ "DSPI_SlaveTransferNonBlocking", "group__dspi__driver.html#ga81f85324750f75b8e7248846c88d99e7", null ],
    [ "DSPI_SlaveWriteData", "group__dspi__driver.html#ga952c2bfcb7e3ac7d3608ec16add273dc", null ],
    [ "DSPI_SlaveWriteDataBlocking", "group__dspi__driver.html#gad7a98ccdb5dcd3ea9c282893b79cee79", null ],
    [ "DSPI_StartTransfer", "group__dspi__driver.html#gac3fb40ea05b407f5b335c0a47330e3a8", null ],
    [ "DSPI_StopTransfer", "group__dspi__driver.html#ga09021ebd27d4ccf5d85398b5bbf12045", null ],
    [ "g_dspiDummyData", "group__dspi__driver.html#ga07177e709a23889fc72566d27715a597", null ]
];