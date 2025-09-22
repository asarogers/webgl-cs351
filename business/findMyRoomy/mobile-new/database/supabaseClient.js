// supabaseClient.js
import AsyncStorage from '@react-native-async-storage/async-storage';
import { createClient } from '@supabase/supabase-js';

const supabaseUrl = process.env.EXPO_PUBLIC_SUPABASE_URL;
const supabaseAnonKey = process.env.EXPO_PUBLIC_SUPABASE_ANON_KEY;

// console.log(supabaseUrl, "called")
// console.log(EXPO_PUBLIC_SUPABASE_ANON_KEY, "called")

export const supabase = createClient(supabaseUrl, supabaseAnonKey, {
  auth: {
    storage: AsyncStorage,
    autoRefreshToken: true,
    persistSession: true,
    detectSessionInUrl: false,
  },
});

// Optional: Database table schemas for TypeScript users
/*
export interface Profile {
  id: string;
  first_name?: string;
  last_name?: string;
  display_name?: string;
  email?: string;
  avatar_url?: string;
  created_at: string;
  updated_at: string;
}
*/