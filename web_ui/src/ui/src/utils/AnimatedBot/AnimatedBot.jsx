import React from 'react'
import { Box } from '@mui/material';

import './botStyle.css';

const AnimatedBot = (props) => {

    const mouthState = props.mouth;
    const faceState = props.face;

  return (
    <Box>
        <Box id="bot" className={faceState === 'thinking' ? "thinking" : "neutral"}>
            <Box id="head">
            <Box id="left-ear">
                <Box id="left-ear-inner"></Box>
            </Box>
            <Box id="face" sx={{backgroundColor:"#25292a",borderColor:"#0000 !important"}}>
                <Box id="eyes">
                <Box id="left-eye"></Box>
                <Box id="right-eye"></Box>
                </Box>
                <Box id={mouthState === "happy"?"smile_mouth":(mouthState === "normal"?"normal_mouth":"sad_mouth")}></Box>
            </Box>
            <Box id="right-ear">
                <Box id="right-ear-inner"></Box>
            </Box>
            </Box>
        </Box>
</Box>
  )
}

export default AnimatedBot