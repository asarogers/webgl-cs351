import { createTheme } from '@mui/material/styles';

export const theme = createTheme({
  components: {
    MuiButton: {
      variants: [
        {
          props: { variant: 'call_to_action' },
          style: {
            background: 'linear-gradient(to top, #FF861D, #FBDF02)',
            color: '#000',
            fontWeight: 'bold',
            textTransform: 'none',
            alignItems: "center", 
            fontSize: "clamp(0.75rem, 1.5vw, 0.8rem)",
            padding: '0.7rem 1.5rem',
            borderRadius: '30px',
            border: 'none',
            margin: 0,
            transition: 'all 0.3s ease', 
            '&:hover': {
              // Gradient transition for hover
              background: 'linear-gradient(to top, #F5C700, #D87814)',
              boxShadow: '0 10px 15px rgba(0, 0, 0, 0.3)', // Softer, larger shadow
              transform: 'scale(1.05)', // Slightly enlarge on hover
              color: '#fff', // Change text color on hover
            },
          },
        },
        {
          props: { variant: 'call_to_action_hover' },
          style: {
            transition: "all 0.3s ease-in-out",
            background: 'rgb(255, 255, 255)',
            color: '#000',
            fontWeight: 'bold',
            textTransform: 'none',
            alignItems: "center", 
            fontSize: "clamp(0.75rem, 1.5vw, 0.8rem)",
            padding: '0.7rem 1.5rem',
            borderRadius: '30px',
            border: 'none',
            margin: 0,
            transition: 'all 0.3s ease', 
            '&:hover': {
              // Gradient transition for hover
              background: 'linear-gradient(to top,rgb(255, 255, 255),rgb(255, 255, 255))',
              boxShadow: '0 10px 15px rgba(0, 0, 0, 0.3)', // Softer, larger shadow
              transform: 'scale(1.05)', // Slightly enlarge on hover
              color: '#fff', // Change text color on hover
            },
          },
        },

        {
          props:{ variant:"ghost_button"},
          style:{
            color: '#fff', 
            background: '#212121',
            fontWeight: 'bold',
            textTransform: 'none',
            alignItems: "center", 
            fontSize: "clamp(0.75rem, 1.5vw, 0.8rem)",
            padding: '0.7rem 1.5rem',
            borderRadius: '30px',
            border: '1px solid rgba(255, 255, 255, 0.33)  ',
            margin: 0,
            transition: 'all 0.3s ease', 
            '&:hover': {
              // Gradient transition for hover
              background: 'white',
              boxShadow: '0 10px 15px rgba(0, 0, 0, 0.3)', // Softer, larger shadow
              transform: 'scale(1.05)', // Slightly enlarge on hover
              color: '#000', 
            },
          }
        }
      ],
    },
  },
});
