import React from 'react'
import { useSelector, useDispatch } from 'react-redux'
import { Box, Button, Container, Stack, Typography } from '@mui/material';
import { useState, useEffect } from 'react'
import {Slider} from '@mui/material';
import { setKP, setKI, setKD } from '../features/PID/PIDSlicer';

const PIDController = () => {

  const rosConnectionState = useSelector((state) => state.RosConnection)
  const PIDState = useSelector((state) => state.PID)
  const dispatch = useDispatch()
  const [cmd_vel,setcmd_vel] = useState()
  const [p_SliderValue,setPSliderValue] = useState(PIDState.KP)
  const [i_SliderValue,setISliderValue] = useState(PIDState.KI)
  const [d_SliderValue,setDSliderValue] = useState(PIDState.KD)

  const [PID,setPID] = useState({KP:p_SliderValue,KI:i_SliderValue,KD:d_SliderValue})

  // ##################### Styling and general Functions ############################# //

const marks = [

  {
    value: 0,
    label: "0"
  },
  {
    value: 5,
    label: "5"
  }
];


const handleSlider = (sliderType, newValue) => {
  switch(sliderType){
    case 'KP': setPSliderValue(newValue);break;
    case 'KI': setISliderValue(newValue);break;
    case 'KD': setDSliderValue(newValue);break;
    default: setPSliderValue(1);break;
  }
}

  const buttonStyles = {
    color:"#fff",
    backgroundColor:"#25292a",
    height:30,
    width:190,
    fontSize:20,
    px:5,
    py:3.5,
    m:2,
    fontWeight:"bold",
    borderRadius:2
}

  const [vel,setVel] = useState({
    linear:{
      x:0,
      y:0,
      z:0
    },
    angular:{
      x:0,
      y:0,
      z:0

    }
  })

  
  // ##################### Throttling Functions ############################# //

  const handleThrottleMessage = () => {
    if(rosConnectionState.ros && rosConnectionState.ros.isConnected && !cmd_vel){
      setcmd_vel(new window.ROSLIB.Topic({
        ros: rosConnectionState.ros,
        name:"/controller/cmd_vel",
        messageType:"geometry_msgs/Twist"
      }))
    }
  }

  const changeSpeedWithDirection = (direction)=>{
    
  switch(direction){
      case 'forward': 
      return { linear:{ x:2,y:0,z:0},
              angular:{x:0,y:0,z:0}}
      case 'backward': 
      return { linear:{ x:-2,y:0,z:0},
              angular:{x:0,y:0,z:0}}

      case 'right': 
      return { linear:{ x:0,y:0,z:0},
              angular:{x:0,y:0,z:-1}}

      case 'left': 
      return { linear:{ x:0,y:0,z:0},
              angular:{x:0,y:0,z:1}}

      case 'stop': 
      return { linear:{ x:0,y:0,z:0},
              angular:{x:0,y:0,z:0}}
      default : 
      return { linear:{ x:0,y:0,z:0},
              angular:{x:0,y:0,z:0}}
}
}

const Move = (direction)=>{
  const speed = changeSpeedWithDirection(direction);
  var twist = new window.ROSLIB.Message(
      speed
    )
  cmd_vel?.publish(twist);
}

// ##################### PID Functions ############################# //

const PID_topic = new window.ROSLIB.Topic({
  ros: rosConnectionState.ros,
  name:"/controller/pid",
  messageType:"depth_controller/PID"
})
const publish_pid = async () =>{

    dispatch(setKP(p_SliderValue))
    dispatch(setKI(i_SliderValue))
    dispatch(setKD(d_SliderValue))
    
    const PID_message = new window.ROSLIB.Message(PID)
    rosConnectionState.ros? PID_topic?.publish(PID_message) : console.log("ros is",rosConnectionState.ros)
    console.log(" PID Published",PID)
}

useEffect(()=>{
  setPID({...PID,KP:p_SliderValue})
},[p_SliderValue])

useEffect(()=>{
  setPID({...PID,KI:i_SliderValue})
},[i_SliderValue])

useEffect(()=>{
  setPID({...PID,KD:d_SliderValue})
},[d_SliderValue])

// ##################### ForceStop function ############################# //

const handleStop = ()=>{
  Move('stop')
}

// ##################### useEffect function ############################# //

useEffect(() => {
  if(! rosConnectionState.ros){
    setcmd_vel(null)
  }
  else if(rosConnectionState.ros.isConnected){
    handleThrottleMessage()
  }

}, [rosConnectionState.ros]);


    return (
      <Container>
        <Container  sx={{justifyContent:"center",display:"flex"}}>

            <Box sx={{mt:9}}>

                  <Stack direction="row" gap={3} sx={{alignItems:"center",mb:11}}>
                      <Typography variant="h4" sx={{width:50,fontWeight:"bold"}}>Kp</Typography>
                      <Slider
                      sx={{width:700}}
                          step={0.1}
                          defaultValue={1}
                          min={0}
                          max={5}
                          value={p_SliderValue}
                          marks= {marks}
                          valueLabelDisplay="on"
                          onChange={(event)=>handleSlider("KP",event.target.value)}
                        />
                  </Stack>

                  <Stack direction="row" gap={3} sx={{alignItems:"center",mb:11}}>
                    <Typography variant="h4" sx={{width:50,fontWeight:"bold"}}>Ki</Typography>
                    <Slider
                    sx={{width:700}}
                        step={0.1}
                        defaultValue={0.2}
                        min={0}
                        max={5}
                        marks= {marks}
                        value={i_SliderValue}
                        valueLabelDisplay="on"
                        onChange={(event)=>handleSlider("KI",event.target.value)}
                      />
                  </Stack>

                  <Stack direction="row" gap={3} sx={{alignItems:"center",mb:11}}>
                    <Typography variant="h4" sx={{width:50,fontWeight:"bold"}}>Kd</Typography>
                    <Slider
                    sx={{width:700}}
                        step={0.1}
                        defaultValue={0}
                        min={0}
                        max={5}
                        // getAriaValueText={valuetext}
                        value={d_SliderValue}
                        marks={marks}
                        valueLabelDisplay="on"
                        onChange={(event)=>handleSlider("KD",event.target.value)}
                      />
                  </Stack>
            </Box>
        </Container>
        <Stack direction={"row"} gap={6} sx={{mx:"auto", alignItems:"center",justifyContent:"center",alignItems:"center"}}>
            <Button variant='contained'  sx={{backgroundColor:"lightgreensea",color:"#fff",mt:0,px:13,py:2,fontSize:19,fontWeight:"bold"}} onClick={()=>publish_pid()}>Publish</Button>
        </Stack>
      </Container>
  
    )
}

export default PIDController
