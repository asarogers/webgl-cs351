import React from 'react';
import { Box, Container, Typography, Grid, Paper, Button, TextField } from '@mui/material';
import CustomLabel from "../../../components/CustomLabel";
import blog1 from "../img/blog1.png"
import blog2 from "../img/articles1.png"
import blog3 from "../img/articles2.png"
import blog4 from "../img/articles3.png"

const BlogSection = () => {
  return (
    <Box sx={{
      color: 'white', padding: '4rem 0', backgroundColor: 'rgba(255, 255, 255, 0.01)'

    }}>
      <Container maxWidth="lg">
        {/* Section Title */}
        <Typography
          variant="h4"
          sx={{ textAlign: 'center', fontWeight: 'bold', color: '#ff9f00' }}
        >
          <CustomLabel label={"Blog"} />
        </Typography>

        <Typography variant="h3" sx={{ textAlign: 'center', marginBottom: '2rem' }}>
          Discover Our Latest  <span style={{
            background: "linear-gradient(to top, #FF861D,#FF861D, rgb(255, 176, 29),#FBDF02, #FBDF02)", // Gradient background
            WebkitBackgroundClip: "text", // For Safari
            backgroundClip: "text", // Standard
            color: "transparent", // Make text color transparent to show gradient
          }}>Articles</span>
        </Typography>

        {/* Blog Articles Section */}
        <Grid container spacing={4} justifyContent="center"

        >
          {/* Featured Article */}
          <Grid item xs={12} md={6}
          >
            <Paper
              sx={{
                boxShadow: "0px 4px 20px rgba(255, 255, 255, 0.1), 0px 0px 5px rgba(255, 255, 255, 0.2) inset",
                border: "1px solid rgba(255, 255, 255, 0.3)",
                padding: '1.5rem',
                backgroundColor: 'rgba(235, 235, 235, 0.05)',
                borderRadius: '15px',
                display: 'flex',
                flexDirection: 'column',
                alignItems: 'center',
                textAlign: 'left',
                transition: 'transform 0.3s',
                '&:hover': {
                  transform: 'scale(1.03)',
                  backgroundColor: '#333',
                },
              }}
            >
              <img src={blog1} alt="Article 1" style={{ width: '100%', borderRadius: '10px' }} />
              <Typography variant="h7" sx={{ color: '#D0D0D0', marginTop: '1rem', textAlign: "start", width: "100%" }}>
                Aug 20, 2022
              </Typography>
              <Typography variant="h5" sx={{ color: 'white', marginTop: '1rem', fontWeight: 'bold' }}>
                A Comprehensive Guide to Navigating and Utilizing Advanced Platform Features
              </Typography>
              <Typography variant="body2" sx={{ color: 'rgba(155, 149, 149, 0.9)', marginBottom: '1rem' }}>
                This guide covers how to effectively navigate and make the most of our platform’s advanced features.
              </Typography>
              <Button variant="contained" color="warning" sx={{ borderRadius: '20px' }}>
                Read More
              </Button>
            </Paper>
          </Grid>

          {/* Side Articles */}
          <Grid item xs={12} md={6} container spacing={2}>
            {[{
              date: "Aug 10, 2022",
              title: "In-Depth Exploration: How to Effectively Manage and Interact with NFTs on Our Platform",
              image: blog2
            }, {
              date: "Aug 5, 2022",
              title: "Maximizing Your Potential: Strategies for Leveraging Interactive Tools and Features on Our Platform",
              image: blog3
            },
            {
              date: "Aug 5, 2022",
              title: "Unlocking the Full Power of Our Platform: Step-by-Step Instructions for Engaging with Key Features",
              image: blog4
            }
          ].map((article, index) => (
              <Grid item xs={12} key={index}>
                <Paper
                  sx={{
                    boxShadow: "0px 4px 20px rgba(255, 255, 255, 0.1), 0px 0px 5px rgba(255, 255, 255, 0.2) inset",
                    border: "1px solid rgba(255, 255, 255, 0.3)",
                    display: 'flex',
                    alignItems: 'center',
                    backgroundColor: 'rgba(235, 235, 235, 0.05)',
                    borderRadius: "12px",
                    padding: '1rem',
                    transition: 'transform 0.3s',
                    '&:hover': {
                      transform: 'scale(1.03)',
                      backgroundColor: '#333',
                    },
                  }}
                >
                  <img src={article.image} alt={article.title} style={{ width: '100px', borderRadius: '10px', marginRight: '1rem' }} />
                  <Box>
                    <Typography variant="body2" sx={{ color: '#D0D0D0' }}>
                      {article.date}
                    </Typography>
                    <Typography variant="h6" sx={{ color: 'white' }}>
                      {article.title}
                    </Typography>
                  </Box>
                </Paper>
              </Grid>
            ))}
          </Grid>
        </Grid>

        {/* Newsletter Subscription Section */}
        
      </Container>
    </Box>
  );
};

export default BlogSection;
