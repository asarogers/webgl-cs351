// AppNavigator.js
import React from "react"
import { createNativeStackNavigator } from "@react-navigation/native-stack"
import onboardingNavigator from "./OnboardingNavigator"
import TabNavigator from "./TabNavigator"


const Stack = createNativeStackNavigator()

const AppNavigator = () =>{
    const isFirstTimeUser = true;

    return(
        <Stack.Navigator screenOptions={{ headerShown: false}}>
            {
                isFirstTimeUser? (
                    <Stack.Screen name="Onboarding" component={onboardingNavigator} />
                ) :(
                    <Stack.Screen name = "Main" component={TabNavigator} />
                    
                )
            }
        </Stack.Navigator>
    )
}