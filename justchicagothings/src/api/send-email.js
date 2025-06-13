import { Resend } from 'resend';

const resend = new Resend(process.env.RESEND_API_KEY);

export default async function handler(req, res) {
  console.log("✅ API hit");

  if (req.method !== 'POST') {
    console.log("❌ Wrong method:", req.method);
    return res.status(405).send('Method not allowed');
  }

  const { name, email } = req.body;
  console.log("📨 Incoming data:", name, email);
  console.log("🔐 API KEY present?", !!process.env.RESEND_API_KEY);

  try {
    const result = await resend.emails.send({
      from: 'onboarding@resend.dev',
      to: 'softwareace.j@gmail.com',
      subject: 'New Newsletter Signup',
      html: `<p><strong>Name:</strong> ${name}</p><p><strong>Email:</strong> ${email}</p>`,
    });

    console.log("✅ Email sent:", result);
    return res.status(200).json({ success: true, result });
  } catch (error) {
    console.error("❌ Email error:", error);
    return res.status(500).json({ error: error.message });
  }
}
