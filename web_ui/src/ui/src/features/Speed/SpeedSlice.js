import { createSlice } from '@reduxjs/toolkit'

const initialState = {
  value: 0,
}

export const SpeedSlice = createSlice({
  name: 'speed',
  initialState,
  reducers: {
    setSpeed: (state,action) => {
      state.value = action.payload
    }
  },
})

export const { setSpeed } = SpeedSlice.actions
export default SpeedSlice.reducer
