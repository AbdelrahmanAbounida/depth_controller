import { createSlice } from '@reduxjs/toolkit'

const initialState = {
    KP:1.2,
    KI:0.3,
    KD: 0.1
  }

export const PIDSlice = createSlice({
  name: 'PID',
  initialState,
  reducers: {
    setKP: (state,action) => {
      state.KP = action.payload
    },
    setKI: (state,action) => {
        state.KI = action.payload
      },
    setKD: (state,action) => {
        state.KD = action.payload
      },
  },
})

export const { setKP,setKI,setKD } = PIDSlice.actions
export default PIDSlice.reducer
