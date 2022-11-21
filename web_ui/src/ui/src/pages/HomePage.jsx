import React from 'react'
import { useSelector, useDispatch } from 'react-redux'
import { startConnection,error,connect, disconnect } from '../features/RosConnection/RosConnectionSlice'
import { Stack, Box, Button } from '@mui/material'
import { useState, useEffect } from 'react'
import {Radio} from 'react-loader-spinner'
import AnimatedBot from '../utils/AnimatedBot/AnimatedBot'


const Home = () => {

  const rosMessageStyle = {backgroundColor:"#25292a",fontSize:27,width:400,py:2,borderRadius:10}
  const rosConnectionState = useSelector((state) => state.RosConnection)
  const rosDispatch = useDispatch()

  const connectionMessages = {
    'connected':
            <Box sx={rosMessageStyle}>{rosConnectionState.connection_message}</Box>,
    'loading':
            <Box sx={{...rosMessageStyle,backgroundColor:"inherit"}}>
                <Radio
                    visible={true}
                    height="80"
                    width="80"
                    ariaLabel="radio-loading"
                    colors = {['lightseagreen','lightseagreen','lightseagreen']}
                    wrapperClass="radio-wrapper"
                  />
            </Box>,
    'notConnected':<Box sx={rosMessageStyle}>{rosConnectionState.connection_message}</Box>,
  }

  
  const handleConnectionMessage = (connection_status)=>{
    switch(connection_status){
      case 'connected': {return connectionMessages['connected'];break;}
      case 'loading': {return connectionMessages['loading'];break;}
      case 'notConnected': {return connectionMessages['notConnected'];break;}
      default: {return connectionMessages['notConnected'];break;}
    }
  }
  const [connectionMessage, setconnectionMessage] = useState(connectionMessages['notConnected']);

  const handleConnect = () =>{
      rosDispatch(startConnection())
  }

  const handleDisconnect = () =>{
      rosDispatch(disconnect())
  }

  useEffect(()=>{
    setconnectionMessage(handleConnectionMessage(rosConnectionState.connection_status))
    if(rosConnectionState.ros && rosConnectionState.connection_status){

      rosConnectionState.ros.on('connection', ()=>{
        setTimeout( ()=>{
          rosDispatch(connect())
        },1000 )
      })
      rosConnectionState.ros.on('error', ()=>{
        rosDispatch(error())
      })
      rosConnectionState.ros.on('close', ()=>{
        rosDispatch(disconnect())
      })
    }
  },[rosConnectionState.connection_status])


  return (
    <Stack sx={{display:"flex", alignItems:"center",justifyContent:"center"}}>

      <Box sx={{position:"relative",height:520,display:"flex",alignItems:"center"}}>
          <AnimatedBot sx={{position:"absolute !important",top:1}} face={rosConnectionState.botState.face} mouth={rosConnectionState.botState.mouth}/>

          <Box sx={{height:30,mb:11,mt:0,position:"absolute",bottom:0,left:-9}}>
            <Box sx={{color:rosConnectionState.connection_color}}>
              {connectionMessage}
            </Box>
          </Box>
      </Box>

      <Stack direction="row" gap={13} sx={{mt:0,mx:"auto"}}>
        <Button 
            variant="contained"  
            sx={{borderRadius:3,color:"#fff",width:210,py:1,fontSize:20,fontWeight:"bold",backgroundColor:"primary.dark","&:hover":{backgroundColor:"primary.main"}}}
            onClick={()=>{handleConnect()}}
            >
            Connect
        </Button>
        <Button 
            variant="contained"  
            sx={{borderRadius:3,color:"#fff",width:210,py:1,fontSize:20,fontWeight:"bold",backgroundColor:"error.dark","&:hover":{backgroundColor:"error.main"}}}
            onClick={()=>{handleDisconnect()}}
            >
            disconnect
        </Button>

      </Stack>

    </Stack>
  )
}

export default Home
