import React from "react";
import { AppBar, Toolbar, Typography, Box, Button, Grid, Container } from "@mui/material";
import Navbar from "../../components/Navbar";
import campusConnex from "../../images/campusConnex.png"
import commercial from "../../images/commercial.mp4"

export default function HeroSection() {
    return (
        <Box sx={{ backgroundColor: "#0a0a0a", color: "#fff", minHeight: "100vh", paddingBottom: "3rem" }}>
            <Container maxWidth="lg">
                <Navbar />
                <Grid container spacing={1} alignItems="center" sx={{ marginTop: "4rem" }}>
                    {/* Left Side - Text Content */}
                    <Grid item xs={12} md={6}>
                        <Box>
                            <Typography
                                variant="overline"
                                sx={{
                                    borderRadius: "25px",
                                    border: "1px solid rgba(255, 255, 255, 1)",
                                    padding: "10px",
                                    fontSize: "1.5rem",
                                    color: "#FFC107",
                                    fontWeight: "bold",
                                    textTransform: "uppercase",
                                    fontSize: "clamp(0.75rem, 1.75vw, 1.75rem)",
                                }}
                            >
                                Aether Tech
                            </Typography>
                            <Typography
                                variant="h3"
                                sx={{ fontWeight: "bold", margin: "1rem 0", lineHeight: "1.2" }}
                            >
                                Expert Mobile App <br />
                                <span style={{ color: "#FFC107" }}>Development</span> for <br />
                                Your Vision
                            </Typography>
                            <Typography variant="body1" sx={{ marginBottom: "2rem", color: "#aaa" }}>
                                We create tailored mobile apps that turn your ideas into innovative solutions for
                                growth.
                            </Typography>
                            <Box>
                                <Button
                                    variant="call_to_action"
                                    sx={{
                                        backgroundColor: "#FFC107",
                                        color: "#000",
                                        marginRight: "1rem",
                                        padding: "0.8rem 2rem",
                                        fontWeight: "bold",
                                        "&:hover": { backgroundColor: "#e6a806" },
                                    }}
                                >
                                    Schedule An Appointment
                                </Button>
                                <Button
                                    variant="ghost_button"
                                >
                                    Learn More
                                </Button>
                            </Box>
                            {/* Ratings */}
                            <Box display="flex" alignItems="center" marginTop="2rem">
                                <Box
                                    component="img"
                                    src={campusConnex}
                                    alt="user-profile"
                                    sx={{
                                        borderRadius: "50%",
                                        width: "40px",
                                        height: "40px",
                                        marginRight: "-10px",
                                        border: "2px solid #fff",
                                    }}
                                />
                                <Box
                                    component="img"
                                    src="https://via.placeholder.com/40"
                                    alt="user-profile"
                                    sx={{
                                        borderRadius: "50%",
                                        width: "40px",
                                        height: "40px",
                                        marginRight: "-10px",
                                        border: "2px solid #fff",
                                    }}
                                />
                                <Box
                                    component="img"
                                    src="https://via.placeholder.com/40"
                                    alt="user-profile"
                                    sx={{
                                        borderRadius: "50%",
                                        width: "40px",
                                        height: "40px",
                                        border: "2px solid #fff",
                                    }}
                                />
                                <Typography variant="body2" sx={{ marginLeft: "1rem", color: "#aaa" }}>
                                    4.9 ⭐ Rating Reviews From <br />
                                    <strong>12k+ People</strong>
                                </Typography>
                            </Box>
                        </Box>
                    </Grid>

                    {/* Right Side - Commercial Video */}
                    <Grid isplay="flex" justifyContent="center">
                        <Box
                            sx={{
                                width: "50vw",
                                maxWidth: "580px", // Adjust as needed
                                borderRadius: "8px",
                                overflow: "hidden", // Ensures rounded corners
                                boxShadow: "0 5px 15px rgba(0, 0, 0, 0.3)", // Optional shadow effect
                            }}
                        >
                            <video
                                src={commercial}
                                autoPlay
                                loop
                                muted
                                playsInline
                                style={{ width: "100%", height: "auto", borderRadius: "8px" }}
                            />
                        </Box>
                    </Grid>

                </Grid>
            </Container>
        </Box>
    );
};
