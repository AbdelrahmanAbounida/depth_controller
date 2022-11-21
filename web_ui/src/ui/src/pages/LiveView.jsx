import React, { useState , useEffect} from 'react'
import {Box,Stack,Slider,Typography, Button} from '@mui/material'
import { Container } from '@mui/system';
import { useSelector, useDispatch } from 'react-redux'
import { setDepth } from '../features/Depth/DepthSlice';

const LiveView = () => {


// ##################### slider Functions ############################# //
const DepthState = useSelector((state) => state.Depth.value)
  const [targetDepth,settargetDepth] = useState(DepthState)
  const marks = [
    {
      value: 0.1,
      label: "0.1"
    },
    {
      value: 0.8,
      label: "0.8"
    }
  ];

// ##################### Depth Functions ############################# //

  const [currentDepth, setCurrentDepth] = useState(-0.1)

  const rosConnectionState = useSelector((state) => state.RosConnection)
  const dispatch = useDispatch()

  // publishing the desired depth
  const depth_publisher = new window.ROSLIB.Topic({
    ros: rosConnectionState.ros,
    name:"/controller/custom_depth_setpoint",
    // name:"/bluerov/custom_depth_setpoint",
    messageType:"std_msgs/Float64"
  })

  const SetTargetDepth = async ()=>{
  dispatch(setDepth(targetDepth))
  const depth_message = new window.ROSLIB.Message({data:targetDepth})
  console.log("target depth is", targetDepth)
    if(depth_publisher.ros){
      depth_publisher.publish(depth_message)
    }
  }

  const depth_listener = new window.ROSLIB.Topic({
    ros: rosConnectionState.ros,
    // name:"/bluerov/ground_truth/state",  // /bluerov/actual_depth
    // messageType:"nav_msgs/Odometry"
    name:"/controller/actual_depth",  // /controller/actual_depth
    messageType:"std_msgs/Float64"
  })

  // subscribing
  const depthCallback = (msg)=>{
    setCurrentDepth(Number(msg.data).toFixed(2))
  }
  // const depthCallback = (msg)=>{
  //   setCurrentDepth(Number(msg?.pose?.pose?.position?.z).toFixed(2))
  // }

  if(depth_listener.ros){
    depth_listener?.subscribe(depthCallback)
  }

  return (
    <Container sx={{}}>
      <Box sx={{mt:9}}>
      <Stack direction="row" gap={3} sx={{alignItems:"center"}}>
        <Typography variant="h4" sx={{width:50,fontWeight:"bold",mr:8,pb:2.3,ml:2}}>Depth</Typography>
        <Slider
          sx={{width:700,pt:2}}
              step={0.1}
              defaultValue={0.6}
              min={0.1}
              max={0.8}
              value={targetDepth*-1}
              marks= {marks}
              valueLabelDisplay="on"
              onChange={(event)=>settargetDepth(event.target.value * -1)}
          />

      </Stack>
      <Button variant='contained'  sx={{backgroundColor:"lightgreensea",color:"#fff",mt:3,px:13,py:2,fontSize:19,fontWeight:"bold", textTransform:"capitalize",mb:9}} onClick={()=>SetTargetDepth()}>Publish Target Depth</Button>
      <Box>
          <Stack direction="row" gap={20} sx={{mx:"auto",justifyContent:"center"}}>
            <Box>
                <Typography variant='h4' sx={{mt:6}}>Current Depth:</Typography>
                <Box sx={{height:30,mb:11,mt:3,backgroundColor:"#25292a",width:300,mx:"auto",py:3.5,color:"lightseagreen",alignItems:"center",display:"flex",justifyContent:"center",fontSize:30,borderRadius:5}}>
                    {currentDepth}
                </Box>
            </Box>
            <Box>
                <Typography variant='h4' sx={{mt:6}}>Target Depth:</Typography>
                <Box sx={{height:30,mb:11,mt:3,backgroundColor:"#25292a",width:300,mx:"auto",py:3.5,color:"lightseagreen",alignItems:"center",display:"flex",justifyContent:"center",fontSize:30,borderRadius:5}}>
                      {DepthState}
                </Box>
          </Box>
          </Stack>
      </Box>
    </Box>
    </Container>
  )
}

export default LiveView
