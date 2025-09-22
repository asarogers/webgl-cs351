import React from "react";
import { createNativeStackNavigator } from "@react-navigation/native-stack";
import SplashScreen from "../screens/onboarding/SplashScreen";
import { WelcomeCarousel } from "../screens/onboarding/welcomeCarousel/WelcomeCarousel";
import RequestLocation from "../screens/onboarding/RequestLocation";
import UserIntentScreen from "../screens/onboarding/UserIntentScreen";
import  LifestyleQuizCarousel  from "../screens/onboarding/vibeQuiz/0_QuizCarousel";

const Stack = createNativeStackNavigator();

const OnboardingNavigator = ({ isFirstTime, completeOnboarding }) => {
  return (
    <Stack.Navigator>
      <Stack.Screen name="Splash" options={{ headerShown: false }}>
        {(props) => <SplashScreen {...props} isFirstTime={isFirstTime} />}
      </Stack.Screen>

      <Stack.Screen
        name="UserIntent"
        component={UserIntentScreen}
        options={{ headerShown: false }}
      />
      <Stack.Screen name="Welcome" options={{ headerShown: false }}>
        {(props) => (
          <WelcomeCarousel {...props} completeOnboarding={completeOnboarding} />
        )}
      </Stack.Screen>

      <Stack.Screen
        name="Quiz"
        component={LifestyleQuizCarousel}
        options={{ headerShown: false }}
      />
      <Stack.Screen
        name="RequestLocation"
        component={RequestLocation}
        options={{ headerShown: false }}
      />
    </Stack.Navigator>
  );
};

export default OnboardingNavigator;
