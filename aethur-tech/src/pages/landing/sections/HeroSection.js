import React from "react";
import { AppBar, Toolbar, Typography, Box, Button, Grid, Container } from "@mui/material";
import backgroundImage from "../img/background1.png"; // Import background image
import campusConnex from "../../../images/campusConnex.png"; //campusConnex.png
import commercial from "../../../images/commercial.mp4"
import CustomLabel from "../../../components/CustomLabel"

export default function HeroSection() {
    return (
        <Box
            sx={{
                position: "relative",
                backgroundColor: "#0a0a0a", // Fallback color
                color: "#fff",
                minHeight: "100%",
                paddingBottom: "3rem",
                backgroundImage: `linear-gradient(rgba(10,10,10,0.25), rgba(10,10,10,0.8)), url(${backgroundImage})`,
                backgroundSize: "cover",
                backgroundPosition: "center",
                backgroundRepeat: "no-repeat",
            }}
        >
        <Grid
            container
            spacing={1}
            alignItems="center"
            sx={{
                marginTop: "4rem",
                justifyContent: { xs: "center", md: "flex-start" },
            }}
        >
            {/* Left Side - Text Content */}
            <Grid
                item
                xs={12}
                md={6}
                sx={{
                    textAlign: { xs: "center", md: "left" },
                    display: "flex",
                    flexDirection: "column",
                    alignItems: { xs: "center", md: "flex-start" },
                    marginBottom: "20px",
                }}
            >
                <Box>
                <CustomLabel label = "AETHER TECH"> </CustomLabel>
                    <Typography
                        variant="h3"
                        sx={{
                            fontWeight: "bold",
                            margin: "1rem 0",
                            lineHeight: "1.2",
                            fontSize: "clamp(1.5rem, 4vw, 3rem)",
                        }}
                    >
                        Expert Mobile App <br />
                        <span style={{ color: "#FFC107" }}>Development</span> for <br />
                        Your Vision
                    </Typography>
                    <Typography
                        variant="body1"
                        sx={{
                            marginBottom: "2rem",
                            color: "#aaa",
                            fontSize: "clamp(0.75rem, 1.5vw, 1.2rem)",
                            maxWidth: { xs: "100%", md: "80%" },
                        }}
                    >
                        We create tailored mobile apps that turn your ideas into innovative solutions for
                        growth.
                    </Typography>
                    <Box
                        sx={{
                            display: "flex",
                            justifyContent: { xs: "center", md: "flex-start" },
                            flexWrap: "wrap",
                            gap: "1rem",
                        }}
                    >
                        <Button
                            variant="call_to_action"
                            sx={{
                                backgroundColor: "#FFC107",
                                color: "#000",
                                padding: "0.8rem 2rem",
                                fontWeight: "bold",
                                "&:hover": { backgroundColor: "#e6a806" },
                            }}
                        >
                            Schedule An Appointment
                        </Button>
                        <Button variant="ghost_button">Learn More</Button>
                    </Box>
                    {/* Ratings */}
                    <Box
                        display="flex"
                        alignItems="center"
                        marginTop="2rem"
                        sx={{
                            justifyContent: { xs: "center", md: "flex-start" },
                        }}
                    >
                        {[campusConnex, "https://via.placeholder.com/40", "https://via.placeholder.com/40"].map(
                            (src, index) => (
                                <Box
                                    key={index}
                                    component="img"
                                    src={src}
                                    alt="user-profile"
                                    sx={{
                                        borderRadius: "50%",
                                        width: "40px",
                                        height: "40px",
                                        marginRight: index < 2 ? "-10px" : "0",
                                        border: "2px solid #fff",
                                    }}
                                />
                            )
                        )}
                        <Typography variant="body2" sx={{ marginLeft: "1rem", color: "#aaa" }}>
                            4.9 ⭐ Rating Reviews From <br />
                            <strong>12k+ People</strong>
                        </Typography>
                    </Box>
                </Box>
            </Grid>

            {/* Right Side - Commercial Video */}
            <Grid
                item
                xs={12}
                md={6}
                sx={{
                    display: "flex",
                    justifyContent: "center",
                    padding: 0,
                    marginTop: { xs: "2rem", md: 0 },
                }}
            >
                <video
                    src={commercial}
                    autoPlay
                    loop
                    muted
                    playsInline
                    style={{
                        minHeight: "360px",
                        width: "100%",
                        height: "auto",
                        borderRadius: "8px",
                    }}
                />
            </Grid>
        </Grid>
        </Box>
    );
}
