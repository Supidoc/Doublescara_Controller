var group__MOTOR__Internal =
[
    [ "_MTR_MotorHandleImpl", "struct__MTR__MotorHandleImpl.html", [
      [ "freewheeling", "struct__MTR__MotorHandleImpl.html#a28cd45c78c5e295687a491838744b053", null ],
      [ "label", "struct__MTR__MotorHandleImpl.html#abc71b3e593c462fc6a3eba4243a90cf8", null ],
      [ "microstep", "struct__MTR__MotorHandleImpl.html#a25782e0bcd0945f0c673b81b6f5d369d", null ],
      [ "previousHoldCurrent", "struct__MTR__MotorHandleImpl.html#ab90efdf6465034866b33110ace0c9a4e", null ],
      [ "reductionFactor", "struct__MTR__MotorHandleImpl.html#a068b88abfa0f8d3412005f860a64932a", null ],
      [ "stepAngle", "struct__MTR__MotorHandleImpl.html#a48ff29dd145331650188df5c94e4d671", null ],
      [ "stepperHandle", "struct__MTR__MotorHandleImpl.html#a2dd7eb698826385c5ab8183999976f67", null ],
      [ "tmcHandle", "struct__MTR__MotorHandleImpl.html#a39fb8a67d47ff090b4ef12f523ad258e", null ]
    ] ],
    [ "_MTR_HandlesArrayItem", "struct__MTR__HandlesArrayItem.html", [
      [ "handle", "struct__MTR__HandlesArrayItem.html#a882bb65eb165305df6ceb8e331df86db", null ],
      [ "used", "struct__MTR__HandlesArrayItem.html#a9194ac3525754d0887986000f11d19a5", null ]
    ] ],
    [ "_MTR_MoveAngleCmdData", "struct__MTR__MoveAngleCmdData.html", [
      [ "angle", "struct__MTR__MoveAngleCmdData.html#a60713860e9c3833e20b2fa11b4297c45", null ]
    ] ],
    [ "_MTR_MoveRevolutionsCmdData", "struct__MTR__MoveRevolutionsCmdData.html", [
      [ "revolutions", "struct__MTR__MoveRevolutionsCmdData.html#a5a758474a2036011e5568d27e5d15a1f", null ]
    ] ],
    [ "_MTR_SetVelocityCmdData", "struct__MTR__SetVelocityCmdData.html", [
      [ "velocity_deg_per_sec", "struct__MTR__SetVelocityCmdData.html#a8c77c490b0a9c4e46fefb2f7b6c1218f", null ]
    ] ],
    [ "_MTR_SetAccelerationCmdData", "struct__MTR__SetAccelerationCmdData.html", [
      [ "acceleration_deg_per_sec2", "struct__MTR__SetAccelerationCmdData.html#aed519396491dd55d7aeb080170152b26", null ]
    ] ],
    [ "_MTR_StopCmdData", "struct__MTR__StopCmdData.html", [
      [ "decelerate", "struct__MTR__StopCmdData.html#a34f285a98eb259dd8d2a6a8af46c8e66", null ]
    ] ],
    [ "_MTR_GetCurrentAngleCmdData", "struct__MTR__GetCurrentAngleCmdData.html", [
      [ "angle", "struct__MTR__GetCurrentAngleCmdData.html#a0b188a4939ab4418e81b98c8e55c6f90", null ]
    ] ],
    [ "_MTR_GetMovementStateCmdData", "struct__MTR__GetMovementStateCmdData.html", [
      [ "state", "struct__MTR__GetMovementStateCmdData.html#a01f7068976d5177c63ef8905998a7736", null ]
    ] ],
    [ "_MTR_SynchronizedMoveCmdData", "struct__MTR__SynchronizedMoveCmdData.html", [
      [ "angles", "struct__MTR__SynchronizedMoveCmdData.html#a2f3ee321e63c0aad6bb7405c107ea8f1", null ],
      [ "count", "struct__MTR__SynchronizedMoveCmdData.html#a333df5a9846301a735f59e6c9ab496c4", null ],
      [ "handles", "struct__MTR__SynchronizedMoveCmdData.html#a8e6e257fb22efae1fc6d6729b2a49081", null ]
    ] ],
    [ "_MTR_SetRunCurrentCmdData", "struct__MTR__SetRunCurrentCmdData.html", [
      [ "current_a", "struct__MTR__SetRunCurrentCmdData.html#a10ce284155c45f7cc920451063d71371", null ]
    ] ],
    [ "_MTR_SetHoldCurrentCmdData", "struct__MTR__SetHoldCurrentCmdData.html", [
      [ "current_a", "struct__MTR__SetHoldCurrentCmdData.html#ac7a4b6259c8c47995b36eff41e0789b9", null ]
    ] ],
    [ "_MTR_InitMotorCmdData", "struct__MTR__InitMotorCmdData.html", [
      [ "config", "struct__MTR__InitMotorCmdData.html#a3026435c2faae4f52b235af3acdd98cf", null ]
    ] ],
    [ "_MTR_EnableFreewheellingCmdData", "struct__MTR__EnableFreewheellingCmdData.html", null ],
    [ "_MTR_DisableFreewheellingCmdData", "struct__MTR__DisableFreewheellingCmdData.html", null ],
    [ "_MTR_CmdQueueItem", "struct__MTR__CmdQueueItem.html", [
      [ "cmdHandle", "struct__MTR__CmdQueueItem.html#ab65a7c8a6273de519b97622b687d539d", null ],
      [ "data", "struct__MTR__CmdQueueItem.html#a6bc6f0baf318d14d8ff2728f6a1c170e", null ],
      [ "deadline", "struct__MTR__CmdQueueItem.html#ab00a40cd2035effbb8efa48e5f0820b5", null ],
      [ "disableFreewheeling", "struct__MTR__CmdQueueItem.html#a70c0e74963ce9437d3d2ad2cae190b9d", null ],
      [ "enableFreewheeling", "struct__MTR__CmdQueueItem.html#a0d095bd3f6553047e93c672a0887b88f", null ],
      [ "getCurrentAngle", "struct__MTR__CmdQueueItem.html#a76557d4f7f7ef2d1f7697e5ae78af11a", null ],
      [ "getMovementState", "struct__MTR__CmdQueueItem.html#a676dee9425369b8760eeae4fc905b93d", null ],
      [ "handle", "struct__MTR__CmdQueueItem.html#a09fabfc5184df5ce60bc804893441261", null ],
      [ "initMotor", "struct__MTR__CmdQueueItem.html#ad7b6643a96c31a9c402abdb11533b398", null ],
      [ "moveAngle", "struct__MTR__CmdQueueItem.html#a7dc129520e7e9ef5d1a7fad51f7522ec", null ],
      [ "moveRevolutions", "struct__MTR__CmdQueueItem.html#a509dc6b75f3f1b5ccb0b20eb43e254d1", null ],
      [ "setAcceleration", "struct__MTR__CmdQueueItem.html#a144d7764b2f14f6566ddfe80cf0de448", null ],
      [ "setHoldCurrent", "struct__MTR__CmdQueueItem.html#a3caf086ca1f7e285d0250dd1f8db44e8", null ],
      [ "setRunCurrent", "struct__MTR__CmdQueueItem.html#a401f24ce0369a7f2787b57a0aaa5daa3", null ],
      [ "setVelocity", "struct__MTR__CmdQueueItem.html#a6dd3632d061f4a65c0fd79cf320566a5", null ],
      [ "stop", "struct__MTR__CmdQueueItem.html#aa69ef6a7505f5fbd14979ecbe0b88af7", null ],
      [ "synchronizedMove", "struct__MTR__CmdQueueItem.html#a317822318f9bca006d19eb3f9f30e7bb", null ],
      [ "type", "struct__MTR__CmdQueueItem.html#a3e443f28693fd8398862eb144e4058a7", null ]
    ] ],
    [ "_MTR_ParallelTaskItem", "struct__MTR__ParallelTaskItem.html", [
      [ "cmdHandles", "struct__MTR__ParallelTaskItem.html#afde001cef0565319bbfd00180c46bc81", null ],
      [ "count", "struct__MTR__ParallelTaskItem.html#abf7a28ae289a0391f509c6aad12aac67", null ],
      [ "returnHandle", "struct__MTR__ParallelTaskItem.html#a5c0d68abd7a8207fd88eb137869d6dbd", null ],
      [ "used", "struct__MTR__ParallelTaskItem.html#acf1af83e1991e3cb5df9adb0ed114875", null ]
    ] ],
    [ "MTR_CMD_QUEUE_SIZE", "group__MOTOR__Internal.html#gaa7fd8bb3bb2d1e660340804f94a66c41", null ],
    [ "MTR_MAX_MOTORS", "group__MOTOR__Internal.html#ga381cc324faff73ce344d43b4267ae441", null ],
    [ "MTR_MAX_NEEDED_CMD_HANDLES", "group__MOTOR__Internal.html#ga6f6ea07736337e5a854b07d725139d7d", null ],
    [ "MTR_MAX_PARALLEL_TASKS", "group__MOTOR__Internal.html#gaa91de8731d1c6ff8e42985bda6ee075f", null ],
    [ "MTR_CmdQueueItem_t", "group__MOTOR__Internal.html#gaa929e5b0a578f350522e7e9f49df706d", null ],
    [ "MTR_CmdType_t", "group__MOTOR__Internal.html#gaae46016790b4a0a8e9f06dd1fe568b66", null ],
    [ "MTR_DisableFreewheellingCmdData_t", "group__MOTOR__Internal.html#gaebdf369004446cf9c3594f17a4ece24f", null ],
    [ "MTR_EnableFreewheellingCmdData_t", "group__MOTOR__Internal.html#gaeef75b1737ec933f9054ee1020df6853", null ],
    [ "MTR_GetCurrentAngleCmdData_t", "group__MOTOR__Internal.html#gae399a58a4ec848e00e2ca58e7287849c", null ],
    [ "MTR_GetMovementStateCmdData_t", "group__MOTOR__Internal.html#gaa3fb881767a12afd15521bda396f9967", null ],
    [ "MTR_HandlesArrayItem_t", "group__MOTOR__Internal.html#ga218226d9b00e4a50e15eb521ed4885c5", null ],
    [ "MTR_InitMotorCmdData_t", "group__MOTOR__Internal.html#ga47dbfc9753882db7b2d44bf2c8f1206d", null ],
    [ "MTR_MotorHandleImpl_t", "group__MOTOR__Internal.html#ga311d45d93ded9212771f08704b6b667a", null ],
    [ "MTR_MoveAngleCmdData_t", "group__MOTOR__Internal.html#ga28ae40cde624abb1ca84fab23693f8a6", null ],
    [ "MTR_MoveRevolutionsCmdData_t", "group__MOTOR__Internal.html#ga80a7a5cd92233713546ee71520010b82", null ],
    [ "MTR_ParallelTaskItem", "group__MOTOR__Internal.html#gad3624e1db311e66b6ac3e2b16afa6a0d", null ],
    [ "MTR_SetAccelerationCmdData_t", "group__MOTOR__Internal.html#ga575d017d9133e9f2c42976d7ffdae5ac", null ],
    [ "MTR_SetHoldCurrentCmdData_t", "group__MOTOR__Internal.html#ga6eb9f10817d0fbf8204861808ababa2c", null ],
    [ "MTR_SetRunCurrentCmdData_t", "group__MOTOR__Internal.html#gae08097ace0352f197d48112da4b6fdba", null ],
    [ "MTR_SetVelocityCmdData_t", "group__MOTOR__Internal.html#ga79e92f54f0433d1978cd5c847ea1be74", null ],
    [ "MTR_StopCmdData_t", "group__MOTOR__Internal.html#gac209c7db389c934d44b4d05f26eeed63", null ],
    [ "MTR_SynchronizedMoveCmdData_t", "group__MOTOR__Internal.html#gac16e87672e9ce1397ba681190ac0a436", null ],
    [ "_MTR_CmdType", "group__MOTOR__Internal.html#ga7e0429af5d2b45a65c693ab8f05fe25b", [
      [ "MTR_CMD_MOVE_ANGLE", "group__MOTOR__Internal.html#gga7e0429af5d2b45a65c693ab8f05fe25ba2c9602e5e60d0efc14a4ae33a35ac77c", null ],
      [ "MTR_CMD_MOVE_ABSOLUTE_ANGLE", "group__MOTOR__Internal.html#gga7e0429af5d2b45a65c693ab8f05fe25ba29c88927276d1e11037c0b3e86d04b16", null ],
      [ "MTR_CMD_MOVE_REVOLUTIONS", "group__MOTOR__Internal.html#gga7e0429af5d2b45a65c693ab8f05fe25ba2571b3ee8605af311842c416e073fc81", null ],
      [ "MTR_CMD_SET_VELOCITY", "group__MOTOR__Internal.html#gga7e0429af5d2b45a65c693ab8f05fe25bab119fcf6892aa8cdbef6a303d35ae4a3", null ],
      [ "MTR_CMD_SET_ACCELERATION", "group__MOTOR__Internal.html#gga7e0429af5d2b45a65c693ab8f05fe25baf5ca6d3184f62edeeb472702e7da06d9", null ],
      [ "MTR_CMD_STOP", "group__MOTOR__Internal.html#gga7e0429af5d2b45a65c693ab8f05fe25bad44f28cb192f415e5d444554e166d120", null ],
      [ "MTR_CMD_GET_CURRENT_ANGLE", "group__MOTOR__Internal.html#gga7e0429af5d2b45a65c693ab8f05fe25ba22370e53c65d689b985804ebb9dd09ff", null ],
      [ "MTR_CMD_GET_MOVEMENT_STATE", "group__MOTOR__Internal.html#gga7e0429af5d2b45a65c693ab8f05fe25ba7ac2d2d83c7ed603808d7c756174c5c2", null ],
      [ "MTR_CMD_SET_HOME_POSITION", "group__MOTOR__Internal.html#gga7e0429af5d2b45a65c693ab8f05fe25bab1ae77b4b34feba2ff51e24e719c376d", null ],
      [ "MTR_CMD_SYNCHRONIZED_MOVE", "group__MOTOR__Internal.html#gga7e0429af5d2b45a65c693ab8f05fe25ba071502104abdcd8d542cff37de225261", null ],
      [ "MTR_CMD_SET_RUN_CURRENT", "group__MOTOR__Internal.html#gga7e0429af5d2b45a65c693ab8f05fe25ba049a0c1c627670961f8b6e5573622b1d", null ],
      [ "MTR_CMD_SET_HOLD_CURRENT", "group__MOTOR__Internal.html#gga7e0429af5d2b45a65c693ab8f05fe25ba957d12a2e7a6dc99c911ac2b66f709bd", null ],
      [ "MTR_CMD_INIT_MOTOR", "group__MOTOR__Internal.html#gga7e0429af5d2b45a65c693ab8f05fe25bad4ccbe599dee3fde4e75813a6ac9671e", null ],
      [ "MTR_CMD_ENABLE_FREEWHEELING", "group__MOTOR__Internal.html#gga7e0429af5d2b45a65c693ab8f05fe25baee58ed4ecea80651efbfd38e1a7ae584", null ],
      [ "MTR_CMD_DISABLE_FREEWHEELING", "group__MOTOR__Internal.html#gga7e0429af5d2b45a65c693ab8f05fe25badb0d765fa89ceb4d0a33eed3f45bc2a9", null ]
    ] ],
    [ "MTRi_process_cmd", "group__MOTOR__Internal.html#ga928b9a79d41efe57d584e7422d5d33e1", null ],
    [ "emergencyStopFlag", "group__MOTOR__Internal.html#gaf1047ff67cd2b16bc743c9b996df0d81", null ],
    [ "motorHandles", "group__MOTOR__Internal.html#gaeea8bff4d30a9428743420d040668cc9", null ],
    [ "mtrCmdHandles", "group__MOTOR__Internal.html#gadfeb8f740ba81e3e261923bb6b4b0251", null ],
    [ "mtrCmdQueue", "group__MOTOR__Internal.html#ga523c31a34713b68f380469e85815e59d", null ],
    [ "parallelTasks", "group__MOTOR__Internal.html#ga4cf2fb96c272e16cfd29187dd8c84e87", null ]
];