import { configureStore } from '@reduxjs/toolkit'
import DrawerOpenReducer from '../features/DrawerOpen/DrawerOpenSlice'
import RosConnectionReducer from '../features/RosConnection/RosConnectionSlice'
import LogoAppearReducer from '../features/LogoAppear/LogoAppearSlice'
import DepthReducer from '../features/Depth/DepthSlice'
import PIDReducer from '../features/PID/PIDSlicer'
import SpeedReducer from '../features/Speed/SpeedSlice'

export const store = configureStore({

  reducer: {
    DrawerOpen:DrawerOpenReducer,
    RosConnection: RosConnectionReducer,
    LogoAppear:LogoAppearReducer,
    Depth: DepthReducer,
    PID: PIDReducer,
    Speed: SpeedReducer,
  },

  middleware: (getDefaultMiddleware) =>
    getDefaultMiddleware({
      serializableCheck: false,
    })
})
