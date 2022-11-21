import React from 'react'
import {Button,Stack, Box,Slider,Typography} from '@mui/material'
import { useSelector, useDispatch } from 'react-redux'
import { setSpeed } from '../features/Speed/SpeedSlice';
import { useState } from 'react';

const Configuration = () => {
  const SpeedState = useSelector((state) => state.Speed.value)
  const rosConnectionState = useSelector((state) => state.RosConnection)
  const [currentSpeed, setCurrentSpeed] = useState(0)
  const dispatch = useDispatch()


  const publishing_topics = {
    up_down : '/bluerov/vertical_thrust',
    right_left: '/bluerov/lateral_thrust',
    forward_backward: '/bluerov/thrust',
    rotate_z: '/bluerov/yaw'
  }

  // up_down topic
  const up_down_publisher = new window.ROSLIB.Topic({
    ros: rosConnectionState.ros,
    name:publishing_topics.up_down,
    messageType:"std_msgs/Float64"
  })

  // right_left topic
  const right_left_publisher = new window.ROSLIB.Topic({
    ros: rosConnectionState.ros,
    name:publishing_topics.right_left,
    messageType:"std_msgs/Float64"
  })

  // forward_backward topic
  const forward_backward_publisher = new window.ROSLIB.Topic({
    ros: rosConnectionState.ros,
    name:publishing_topics.forward_backward,
    messageType:"std_msgs/Float64"
  })

  // rotate_z topic
  const rotate_z_publisher = new window.ROSLIB.Topic({
    ros: rosConnectionState.ros,
    name:publishing_topics.rotate_z,
    messageType:"std_msgs/Float64"
  })

  const marks = [
    {
      value: 0,
      label: "0"
    },
    {
      value: 1,
      label: "1"
    }
  ];
  const zero_message = new window.ROSLIB.Message({data:0})

  const publishTargetSpeed = (direction,factor) =>{

    dispatch(setSpeed(currentSpeed))

    up_down_publisher.publish(zero_message)
    right_left_publisher.publish(zero_message)
    forward_backward_publisher.publish(zero_message)
    rotate_z_publisher.publish(zero_message)

    // speed message
  const speed_message = new window.ROSLIB.Message({data:currentSpeed*factor})
  if(rosConnectionState.connected){
    
    switch(direction){
      case 'up_down': up_down_publisher.publish(speed_message);break;
      case 'right_left': right_left_publisher.publish(speed_message);break;
      case 'forward_backward': forward_backward_publisher.publish(speed_message);break;
      case 'rotate_z': rotate_z_publisher.publish(speed_message);break;
      case 'stop': break;
      default: break;
    }
  }

  } 

  return (

    <Box sx={{mt:9}}>
    <Stack direction="row" gap={3} sx={{alignItems:"center"}}>
        <Typography variant="h4" sx={{width:50,fontWeight:"bold",mr:8,pb:2.3,ml:2}}>Speed</Typography>
        <Slider
          sx={{width:700,pt:2}}
              step={0.1}
              defaultValue={0.1}
              min={0}
              max={1}
              value={currentSpeed}
              marks= {marks}
              valueLabelDisplay="on"
              onChange={(event)=>setCurrentSpeed(event.target.value)}
          />

      </Stack>

      <Box sx={{mx:"auto",width:500}}>
          <Stack direction={"row"} sx={{mx:"auto"}} gap={1}>
            <Button contained="variant" sx={{backgroundColor:"lightseagreen","&:hover":{backgroundColor:"primary.main"},color:"#fff",width:250,py:1.9,fontSize:19,fontWeight:"bold", textTransform:"capitalize",mb:3}} onClick={() => publishTargetSpeed("up_down",1)}>Up</Button>
            <Button contained="variant" sx={{backgroundColor:"lightseagreen","&:hover":{backgroundColor:"primary.main"},color:"#fff",width:250,py:2,fontSize:19,fontWeight:"bold", textTransform:"capitalize",mb:3}} onClick={() => publishTargetSpeed("up_down",-1)}>Down</Button>
          </Stack>

          <Stack direction={"row"} sx={{mx:"auto"}} gap={1}>
              <Button contained="variant" sx={{backgroundColor:"lightseagreen","&:hover":{backgroundColor:"primary.main"},color:"#fff",width:250,py:2,fontSize:19,fontWeight:"bold", textTransform:"capitalize",mb:3}} onClick={() => publishTargetSpeed("right_left",1)}>Right</Button>
              <Button contained="variant" sx={{backgroundColor:"lightseagreen","&:hover":{backgroundColor:"primary.main"},color:"#fff",width:250,py:2,fontSize:19,fontWeight:"bold", textTransform:"capitalize",mb:3}} onClick={() => publishTargetSpeed("right_left",-1)}>Left</Button>
          </Stack>

          <Stack direction={"row"} sx={{mx:"auto"}} gap={1}>
              <Button contained="variant" sx={{backgroundColor:"lightseagreen","&:hover":{backgroundColor:"primary.main"},color:"#fff",width:250,py:2,fontSize:19,fontWeight:"bold", textTransform:"capitalize",mb:3}} onClick={() => publishTargetSpeed("forward_backward",1)}>Forward</Button>
              <Button contained="variant" sx={{backgroundColor:"lightseagreen","&:hover":{backgroundColor:"primary.main"},color:"#fff",width:250,py:2,fontSize:19,fontWeight:"bold", textTransform:"capitalize",mb:3}} onClick={() => publishTargetSpeed("forward_backward",-1)}>Backward</Button>
          </Stack>

          <Stack direction={"row"} sx={{mx:"auto"}} gap={1}>
              <Button contained="variant" sx={{backgroundColor:"lightseagreen","&:hover":{backgroundColor:"primary.main"},color:"#fff",width:250,py:2,fontSize:19,fontWeight:"bold", textTransform:"capitalize",mb:3}} onClick={() => publishTargetSpeed("rotate_z",1)}>Rotate CW</Button>
              <Button contained="variant" sx={{backgroundColor:"lightseagreen","&:hover":{backgroundColor:"primary.main"},color:"#fff",width:250,py:2,fontSize:19,fontWeight:"bold", textTransform:"capitalize",mb:3}} onClick={() => publishTargetSpeed("rotate_z",-1)}>Rotate CCW</Button>
          </Stack>
          <Button contained="variant" sx={{backgroundColor:"error.main","&:hover":{backgroundColor:"error.dark"},color:"#fff",width:250,py:2,fontSize:19,fontWeight:"bold", textTransform:"capitalize",mb:3}} onClick={() => publishTargetSpeed("stop",0)}>Stop</Button>

      </Box>


      </Box>
  )
}

export default Configuration
