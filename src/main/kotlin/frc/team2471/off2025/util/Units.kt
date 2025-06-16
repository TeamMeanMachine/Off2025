@file:Suppress("UNUSED")
package frc.team2471.off2025.util

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.units.measure.*
import kotlin.math.*


//Unit Conversions

//Distance
inline val Distance.asInches: Double get() = `in`(Inches)
inline val Distance.asFeet: Double get() = `in`(Feet)
inline val Distance.asMeters: Double get() = `in`(Meters)
inline val Distance.asCentimeters: Double get() = `in`(Centimeters)
inline val Distance.asMillimeters: Double get() = `in`(Millimeter)

inline val Double.inches: Distance get() = Inches.of(this)
inline val Double.feet: Distance get() = Feet.of(this)
inline val Double.meters: Distance get() = Meters.of(this)
inline val Double.centimeters: Distance get() = Centimeters.of(this)
inline val Double.millimeters: Distance get() = Millimeters.of(this)


//Angle
inline val Angle.asDegrees: Double get() = `in`(Degrees)
inline val Angle.asRotations: Double get() = `in`(Rotations)
inline val Angle.asRadians: Double get() = `in`(Radians)
inline val Angle.asRotation2d: Rotation2d get() = Rotation2d(this)

inline val Double.degrees: Angle get() = Degrees.of(this)
inline val Double.rotations: Angle get() = Rotations.of(this)
inline val Double.radians: Angle get() = Radians.of(this)


//Time
inline val Time.asSeconds: Double get() = `in`(Seconds)
inline val Time.asMinutes: Double get() = `in`(Minutes)
inline val Time.asMilliseconds: Double get() = `in`(Milliseconds)
inline val Time.asMicroseconds: Double get() = `in`(Microseconds)

inline val Double.seconds: Time get() = Seconds.of(this)
inline val Double.milliseconds: Time get() = Milliseconds.of(this)
inline val Double.microseconds: Time get() = Microseconds.of(this)
inline val Double.minutes: Time get() = Minutes.of(this)


//Linear Velocity
inline val LinearVelocity.asInchesPerSecond: Double get() = `in`(InchesPerSecond)
inline val LinearVelocity.asFeetPerSecond: Double get() = `in`(FeetPerSecond)
inline val LinearVelocity.asMetersPerSecond: Double get() = `in`(MetersPerSecond)

inline val Distance.perSecond: LinearVelocity get() = InchesPerSecond.of(this.asInches)

inline val Double.inchesPerSecond: LinearVelocity get() = InchesPerSecond.of(this)
inline val Double.feetPerSecond: LinearVelocity get() = FeetPerSecond.of(this)
inline val Double.metersPerSecond: LinearVelocity get() = MetersPerSecond.of(this)


//Angular Velocity
inline val AngularVelocity.asDegreesPerSecond: Double get() = `in`(DegreesPerSecond)
inline val AngularVelocity.asRotationsPerSecond: Double get() = `in`(RotationsPerSecond)
inline val AngularVelocity.asRPM: Double get() = `in`(RPM)
inline val AngularVelocity.asRadiansPerSecond: Double get() = `in`(RadiansPerSecond)

inline val Angle.perSecond: AngularVelocity get() = DegreesPerSecond.of(this.asDegrees)

inline val Double.degreesPerSecond: AngularVelocity get() = DegreesPerSecond.of(this)
inline val Double.rotationsPerSecond: AngularVelocity get() = RotationsPerSecond.of(this)
inline val Double.rpm: AngularVelocity get() = RPM.of(this)
inline val Double.radiansPerSecond: AngularVelocity get() = RadiansPerSecond.of(this)


//Linear Acceleration
inline val LinearAcceleration.asFeetPerSecondPerSecond: Double get() = `in`(FeetPerSecondPerSecond)
inline val LinearAcceleration.asMetersPerSecondPerSecond: Double get() = `in`(MetersPerSecondPerSecond)
inline val LinearAcceleration.asGs: Double get() = `in`(Gs)

inline val Distance.perSecondPerSecond: LinearAcceleration get() = FeetPerSecondPerSecond.of(this.asFeet)
inline val LinearVelocity.perSecond: LinearAcceleration get() = FeetPerSecondPerSecond.of(this.asFeetPerSecond)

inline val Double.asInchesPerSecondPerSecond: LinearAcceleration get() = InchesPerSecond.per(Second).of(this)
inline val Double.feetPerSecondPerSecond: LinearAcceleration get() = FeetPerSecondPerSecond.of(this)
inline val Double.metersPerSecondPerSecond: LinearAcceleration get() = MetersPerSecondPerSecond.of(this)
inline val Double.Gs: LinearAcceleration get() = Units.Gs.of(this)


//Angular Acceleration
inline val AngularAcceleration.asDegreesPerSecondPerSecond: Double get() = `in`(DegreesPerSecondPerSecond)
inline val AngularAcceleration.asRotationsPerSecondPerSecond: Double get() = `in`(RotationsPerSecondPerSecond)
inline val AngularAcceleration.asRadiansPerSecondPerSecond: Double get() = `in`(RadiansPerSecondPerSecond)

inline val Angle.perSecondPerSecond: AngularAcceleration get() = DegreesPerSecondPerSecond.of(this.asDegrees)
inline val AngularVelocity.perSecond: AngularAcceleration get() = DegreesPerSecondPerSecond.of(this.asDegreesPerSecond)

inline val Double.degreesPerSecondPerSecond: AngularAcceleration get() = DegreesPerSecondPerSecond.of(this)
inline val Double.rotationsPerSecondPerSecond: AngularAcceleration get() = RotationsPerSecondPerSecond.of(this)
inline val Double.radiansPerSecondPerSecond: AngularAcceleration get() = RadiansPerSecondPerSecond.of(this)


//Mass
inline val Mass.asKilograms: Double get() = `in`(Kilograms)
inline val Mass.asGrams: Double get() = `in`(Grams)
inline val Mass.asPounds: Double get() = `in`(Pounds)
inline val Mass.asOunces: Double get() = `in`(Ounces)

inline val Double.kilograms: Mass get() = Kilograms.of(this)
inline val Double.grams: Mass get() = Grams.of(this)
inline val Double.pounds: Mass get() = Pounds.of(this)
inline val Double.ounces: Mass get() = Ounces.of(this)


//The Force
inline val Force.asNewtons: Double get() = `in`(Newtons)
inline val Force.asOuncesForce: Double get() = `in`(OuncesForce)
inline val Force.asPoundsForce: Double get() = `in`(PoundsForce)

inline val Double.newtons: Force get() = Newtons.of(this)
inline val Double.ouncesForce: Force get() = OuncesForce.of(this)
inline val Double.poundsForce: Force get() = PoundsForce.of(this)


//Torque
inline val Torque.asNewtonMeters: Double get() = `in`(NewtonMeters)
inline val Torque.asPoundFeet: Double get() = `in`(PoundFeet)
inline val Torque.asOunceInches: Double get() = `in`(OunceInches)

inline val Double.newtonMeters: Torque get() = NewtonMeters.of(this)
inline val Double.poundFeet: Torque get() = PoundFeet.of(this)
inline val Double.ounceInches: Torque get() = OunceInches.of(this)


//MOI
inline val Double.kilogramSquareMeters: MomentOfInertia get() = KilogramSquareMeters.of(this)

inline val MomentOfInertia.asKilogramSquareMeters: Double get() = `in`(KilogramSquareMeters)


//Voltage
inline val Double.volts: Voltage get() = Volts.of(this)

inline val Voltage.asVolts: Double get() = `in`(Volts)


//Current
inline val Current.asAmps: Double get() = `in`(Amps)

inline val Double.amps: Current get() = Amps.of(this)


//Other
inline val Double.voltsPerSecond: Velocity<VoltageUnit> get() = Volts.per(Second).of(this)

inline val Velocity<VoltageUnit>.asVoltsPerSecond: Double get() = `in`(Volts.per(Second))


//Formulas
fun LinearVelocity.toAngular(radius: Distance) = RadiansPerSecond.of(this.asMetersPerSecond / radius.asMeters)!!
fun AngularVelocity.toLinear(radius: Distance) = MetersPerSecond.of(this.asRadiansPerSecond * radius.asMeters)!!


/**Converts a [Double] in hertz into an equivalent [Time] unit.*/
fun Double.hertzToTime() = if (this == 0.0) 0.0.seconds else (1.0 / this).seconds

@JvmName("sinOf")
fun sin(angle: Angle) = sin(angle.asRadians)
@JvmName("cosOf")
fun cos(angle: Angle) = cos(angle.asRadians)
@JvmName("tanOf")
fun tan(angle: Angle) = tan(angle.asRadians)
fun Angle.sin() = sin(this)
fun Angle.cos() = cos(this)
fun Angle.tan() = tan(this)

fun asin(value: Double) = kotlin.math.asin(value).radians
fun acos(value: Double) = kotlin.math.acos(value).radians
fun atan(value: Double) = kotlin.math.atan(value).radians

fun atan2(y: Double, x: Double) = kotlin.math.atan2(y, x).radians
fun atan2(y: Distance, x: Distance) = kotlin.math.atan2(y.asInches, x.asInches).radians

fun Angle.wrap() = asDegrees.IEEErem(360.0).degrees
fun Angle.unWrap(nearByAngle: Angle) = nearByAngle + (this - nearByAngle).wrap()

fun Angle.absoluteValue() = asDegrees.absoluteValue.degrees

fun Angle.asRotation2d() = Rotation2d(this.asRadians)

//String
fun Angle.toReadableString() = "$asDegrees degrees"
fun Distance.toReadableString() = "$asFeet feet"
fun AngularVelocity.toReadableString() = "$asDegreesPerSecond degrees/second"
fun LinearVelocity.toReadableString() = "$asFeetPerSecond feet/second"
fun AngularAcceleration.toReadableString() = "$asDegreesPerSecondPerSecond degrees/second^2"
fun LinearAcceleration.toReadableString() = "$asFeetPerSecondPerSecond feet/second^2"
fun Time.toReadableString() = "$asSeconds seconds"
fun Voltage.toReadableString() = "$asVolts volts"


//Extra units
//inline val Translation2d.inches: Translation2d get() = Translation2d(this.x.inches, this.y.inches)
//inline val Translation2d.inches: Translation2d get() = Translation2d(this.x.inches, this.y.inches)





