import {Typography} from "@mui/material";

export default function CustomLabel({label}){
    return (
        <Typography
            variant="overline"
            sx={{
                borderRadius: "25px",
                border: "1px solid rgba(255, 255, 255, 1)",
                padding: "0px 4px 0px 4px",
                color: "#FFC107",
                fontWeight: "bold",
                textTransform: "uppercase",
                fontSize: "clamp(0.75rem, 1.75vw, 1.75rem)",
                display: "inline-block",
            }}
        >
            {label}
        </Typography>
    )
}