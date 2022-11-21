import { createSlice } from '@reduxjs/toolkit'

const initialState = {
  value: -0.6,
}

export const DepthSlice = createSlice({
  name: 'depth',
  initialState,
  reducers: {
    setDepth: (state,action) => {
      state.value = action.payload
    }
  },
})

export const { setDepth } = DepthSlice.actions
export default DepthSlice.reducer
