import {Typography} from "@mui/material";

export default function CustomLabel({label}){
    return (
        <Typography
            variant="overline"
            sx={{
                background: "linear-gradient(to top, #FF861D,rgb(255, 119, 0), #FBDF02, #FBDF02)",
                WebkitBackgroundClip: "text",
                borderRadius: "25px",
                border: "1px solid rgba(255, 255, 255, 1)",
                backgroundClip: "text", // Standard
                color: "transparent",
                padding: "0px 6px 0px 6px",
                margin: "0px",
                fontWeight: "bold",
                textTransform: "none",
                fontSize: "clamp(0.55rem, 1.45vw, 1.35rem)",
                display: "inline-block",
            }}
        >
            {label}
        </Typography>
    )
}