import React from 'react';
import { createNativeStackNavigator } from '@react-navigation/native-stack';
import SplashScreen from '../screens/onboarding/SplashScreen';
import WelcomeCarousel from '../screens/onboarding/WelcomeCarousel';
import JointApplicationConfirm from '../screens/housing/JointApplicationConfirm'; // for testing

const Stack = createNativeStackNavigator();

const OnboardingNavigator = () => {
  return (
    <Stack.Navigator>
      <Stack.Screen name="Splash" component={SplashScreen} options={{ headerShown: false }} />
      <Stack.Screen name="Welcome" component={WelcomeCarousel} />
      <Stack.Screen name="JointConfirm" component={JointApplicationConfirm} />
    </Stack.Navigator>
  );
};

export default OnboardingNavigator;
