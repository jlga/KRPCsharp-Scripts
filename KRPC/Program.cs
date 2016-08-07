using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using KRPC.Client.Services.KRPC;
using KRPC.Client.Services.SpaceCenter;
using KRPC.Client.Attributes;
using System.Net;
using KRPC.Client;

namespace KRPC
{
    class Program
    {
        static void Main(string[] args)
        {
            Program p = new Program();
        }

        Connection conn;
        Vessel vessel;
        

        Program()
        {
            conn = new Connection(name: "OrbitLauncher");
            var krpc = conn.KRPC();
            System.Console.WriteLine(krpc.GetStatus().Version);
            Launch();
        }

        public void Launch()
        {
            double turnstartaltitude = 250;
            double turnendaltitude = 45000;
            double targetaltitude = 150000;
            var spaceCenter = conn.SpaceCenter();
            vessel = spaceCenter.ActiveVessel;
            var flight = vessel.Flight();

            // Set up Streams for Telemetry
            var ut = conn.AddStream(() => spaceCenter.UT);
            var altitude = conn.AddStream(() => flight.MeanAltitude);
            var apoapsis = conn.AddStream(() => vessel.Orbit.ApoapsisAltitude);
            var periapsis = conn.AddStream(() => vessel.Orbit.PeriapsisAltitude);
            var eccentricity = conn.AddStream(() => vessel.Orbit.Eccentricity);
            var stage2resources = vessel.ResourcesInDecoupleStage(2, false);
            var stage3resources = vessel.ResourcesInDecoupleStage(3, false);
            var srbfuel = conn.AddStream(() => stage3resources.Amount("SolidFuel"));
            var launcherfuel = conn.AddStream(() => stage2resources.Amount("LiquidFuel"));

            // Pre Launch Setup
            vessel.Control.SAS = false;
            vessel.Control.RCS = false;
            vessel.Control.Throttle = 1;

            // Countdown...
            print("3");
            Thread.Sleep(1000);
            print("2");
            Thread.Sleep(1000);
            print("1");
            Thread.Sleep(1000);
            print("Launch");

            // Activate the first stage
            vessel.Control.ActivateNextStage();
            vessel.AutoPilot.Engage();
            vessel.AutoPilot.TargetPitchAndHeading(90, 90);

            // Main ascent loop
            bool srbs_separated = false;
            double turn_angle = 0;
            while(true)
            {
                // Gravity Turn
                if(turnstartaltitude < altitude.Get() && altitude.Get() < turnendaltitude)
                {
                    double frac = (altitude.Get() - turnstartaltitude) /(turnendaltitude - turnstartaltitude);
                    double newturnangle = frac * 90;
                    if(Math.Abs(newturnangle -turn_angle)> 0.5)
                    {
                        turn_angle = newturnangle;
                        vessel.AutoPilot.TargetPitchAndHeading(90 - (float)turn_angle, 90);
                    }
                }

                // Separate SRBs when empty
                if(!srbs_separated && srbfuel.Get() < 0.1)
                {
                    vessel.Control.ActivateNextStage();
                    srbs_separated = true;
                    print("SRBs separated!");
                }

                // Decrease Throttle when aproaching target apoapsis
                if(apoapsis.Get() > targetaltitude * 0.9)
                {
                    print("Approaching target apoapsis");
                    break;
                }
            }

            // Disable Engines when target apoapsis is reached
            vessel.Control.Throttle = 0.25f;
            while(apoapsis.Get() < targetaltitude)
            {
                continue;
            }
            print("Target apoapsis reached!");
            vessel.Control.Throttle = 0;

            // Wait until out of atmosphere
            print("Coasting out of atmosphere");
            while(altitude.Get() < 70500)
            {
                continue;
            }

            // Plan circularization burn (using vis-viva equation)
            print("Planning circularization burn");
            double mu = vessel.Orbit.Body.GravitationalParameter;
            double r = vessel.Orbit.Apoapsis;
            double a1 = vessel.Orbit.SemiMajorAxis;
            double a2 = r;
            double v1 = Math.Sqrt(mu * ((2 / r) - (1 / a1)));
            double v2 = Math.Sqrt(mu * ((2 / r) - (1 / a2)));
            double delta_v = v2 - v1;
            Node node = vessel.Control.AddNode(ut.Get() + vessel.Orbit.TimeToApoapsis, (float)delta_v);

            //Calculate burn (using rocket equation)
            double F = vessel.AvailableThrust;
            double Isp = vessel.VacuumSpecificImpulse * 9.82;
            double m0 = vessel.Mass;
            double m1 = m0 / Math.Exp(delta_v / Isp);
            double flow_rate = F / Isp;
            double burntime = (m0 - m1) / flow_rate;

            // Orientate ship
            print("Orientating ship for circularization burn");
            vessel.AutoPilot.ReferenceFrame = node.ReferenceFrame;
            vessel.AutoPilot.TargetDirection = Tuple.Create(0.0, 1.0, 0.0);
            vessel.AutoPilot.Wait();

            // Wait until burn
            print("Waiting until circularization burn");
            double burn_ut = ut.Get() + vessel.Orbit.TimeToApoapsis - (burntime / 2);
            double lead_time = 5;
            spaceCenter.WarpTo(burn_ut - lead_time);

            // Execute burn
            print("Ready to execute burn");
            var time_to_apoapsis = conn.AddStream(() => vessel.Orbit.TimeToApoapsis);
            while (time_to_apoapsis.Get() - (burntime / 2.0) > 0)
            {
                continue;
            }
            print("Executing burn" + burntime);
            vessel.Control.Throttle = 1;
            Thread.Sleep(((int)((burntime-0.1)*1000)));
            print("Fine tuning");
            vessel.Control.Throttle = 0.05f;
            var remaining_burn = conn.AddStream(() => node.RemainingBurnVector(node.ReferenceFrame));
            while(remaining_burn.Get().Item2 > 0)
            {
                continue;
            }
            vessel.Control.Throttle = 0;
            node.Remove();

            print("Launch Complete!");




        }

        public void print(string str)
        {
            Console.Out.WriteLine(str);
        }
    }
}
