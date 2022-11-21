import { createSlice } from '@reduxjs/toolkit'

const rosbridge_address =  'ws://0.0.0.0:9090';
const initialState = {
  ros: null,
  connected: false,
  botState: {mouth:"sad",face:"normal"}, // {mouth:"sad",face:"normal"}, {mouth:"normal",face:"thinking"}, { mouth:"sad",face:"neutral"}
  connection_status: 'notConnected', // loading, connected
  connection_color: "#f00",  // lightseagreen, 
  connection_message : "Not Connected"
}
// we have 5 states: (1) start connection (2) connectionLoading (3) connected (4) disconnect (5) error

export const RosConnectionSlice = createSlice({
  name: 'rosConnection',
  initialState,
  reducers: {
    startConnection: (state) => {
          state.ros = new window.ROSLIB.Ros({
            url: rosbridge_address
          })
          state.botState = {mouth:"normal",face:"thinking"}
          state.connection_status = "loading"
          state.connection_color = "lightseagreen"
          state.connection_message = ""
    },
    connect: (state) =>{
      state.connected = true;
      state.botState = {mouth:"happy",face:"thinking"}
      state.connection_status = 'connected';
      state.connection_color = "lightseagreen"
      state.connection_message = "Connected"
    },
    error: (state) =>{
      state.ros=null
      state.connected = false;
      state.botState = {mouth:"sad",face:"neutral"}
      state.connection_status = 'notConnected';
      state.connection_color = "#f00"
      state.connection_message = "Connection Failed"
    },
    disconnect: (state) =>{
      if(state.ros){
        state.ros.close()
        state.ros=null;
      }
      state.connected = false;
      state.botState = {mouth:"sad",face:"neutral"}
      state.connection_status = 'notConnected';
      state.connection_color = "#f00"
      state.connection_message = "Not Connected"
    }
  }
})

export const { startConnection,error, connect,disconnect } = RosConnectionSlice.actions
export default RosConnectionSlice.reducer
