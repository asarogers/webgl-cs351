import { createNativeStackNavigator } from '@react-navigation/native-stack';
// import WelcomeCarousel from '../screens/WelcomeCarousel';
import LoginScreen from '../screens/auth/LoginScreen';
import SignupScreen from '../screens/auth/SignUpScreen';
// import ForgotPassword from '../screens/ForgotPassword';
// import ResetPassword from '../screens/ResetPassword';
import PhoneAndEmailVerificationScreen from '../screens/auth/PhoneAndEmailVerificationScreen';

const Stack = createNativeStackNavigator();

export default function AuthNavigator() {
  return (
    <Stack.Navigator screenOptions={{ headerShown: false }}>
      <Stack.Screen name="Login" component={LoginScreen} />
      <Stack.Screen name="SignUp" component={SignupScreen} />
      <Stack.Screen name="PhoneAndEmailVerificationScreen" component={PhoneAndEmailVerificationScreen} />
      {/* <Stack.Screen name="ForgotPassword" component={ForgotPassword} />
      <Stack.Screen name="ResetPassword" component={ResetPassword} /> */}
    </Stack.Navigator>
  );
}
