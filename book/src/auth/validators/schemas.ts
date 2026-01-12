import { z } from 'zod';

// Email validation schema
export const emailSchema = z.string().email('Please enter a valid email address');

// Password validation schema with complexity requirements
export const passwordSchema = z
  .string()
  .min(8, 'Password must be at least 8 characters')
  .regex(/.*[a-z].*/, 'Password must contain at least one lowercase letter')
  .regex(/.*[A-Z].*/, 'Password must contain at least one uppercase letter')
  .regex(/.*\d.*/, 'Password must contain at least one number');

// Software background enum schema
export const softwareBackgroundSchema = z.enum([
  'Beginner',
  'Intermediate',
  'Advanced',
  'Expert'
], {
  required_error: 'Please select your software experience level',
  invalid_type_error: 'Invalid software experience level',
});

// Hardware background enum schema
export const hardwareBackgroundSchema = z.enum([
  'No Experience',
  'Basic Knowledge',
  'Hands-on Experience',
  'Professional Experience'
], {
  required_error: 'Please select your hardware experience level',
  invalid_type_error: 'Invalid hardware experience level',
});

// Signup schema combining all validations
export const signupSchema = z
  .object({
    email: emailSchema,
    password: passwordSchema,
    confirmPassword: z.string(),
    software_background: softwareBackgroundSchema,
    hardware_background: hardwareBackgroundSchema,
  })
  .refine((data) => data.password === data.confirmPassword, {
    message: 'Passwords do not match',
    path: ['confirmPassword'],
  });

// Signin schema
export const signinSchema = z.object({
  email: emailSchema,
  password: z.string().min(1, 'Password is required'),
});

// Export types for use in components
export type SignupData = z.infer<typeof signupSchema>;
export type SigninData = z.infer<typeof signinSchema>;