// HomeScreen.js
import React, { useEffect, useState } from 'react';
import { View, ActivityIndicator, StyleSheet } from 'react-native';
import AppNavigator from '../src/navigation/AppNavigator';
import authService from '@/database/authService';

export default function HomeScreen() {
  const [isLoading, setIsLoading] = useState(true);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [userOnboarding, setUserOnboarding] = useState(null);

  useEffect(() => {
    initializeAuth();
    
    // Listen for auth state changes
    const { data: { subscription } } = authService.onAuthStateChange(
      async (event, session) => {
        console.log('Auth state changed:', event, !!session);
        
        if (event === 'SIGNED_IN' && session) {
          setIsAuthenticated(true);
          const onboarding = await authService.getOnboardingStep(session.user.id);
          setUserOnboarding(onboarding);
        } else if (event === 'SIGNED_OUT') {
          setIsAuthenticated(false);
          setUserOnboarding(null);
        }
        
        setIsLoading(false);
      }
    );

    return () => {
      subscription?.unsubscribe();
    };
  }, []);

  const initializeAuth = async () => {
    try {
      const { success, session } = await authService.getCurrentSession();
      
      if (success && session) {
        console.log('Found existing session for user:', session.user.email);
        setIsAuthenticated(true);
        const onboarding = await authService.getOnboardingStep(session.user.id);
        setUserOnboarding(onboarding);
      } else {
        console.log('No existing session found');
        setIsAuthenticated(false);
      }
    } catch (error) {
      console.error('Auth initialization failed:', error);
      setIsAuthenticated(false);
    } finally {
      setIsLoading(false);
    }
  };

  if (isLoading) {
    return (
      <View style={styles.loadingContainer}>
        <ActivityIndicator size="large" color="#3B82F6" />
      </View>
    );
  }

  // Remove NavigationContainer from here
  return (
    <AppNavigator 
      isAuthenticated={isAuthenticated}
      userOnboarding={userOnboarding}
    />
  );
}

const styles = StyleSheet.create({
  loadingContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    backgroundColor: '#111827',
  },
});