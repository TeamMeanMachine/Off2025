package frc.team2471.off2025.util

import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.units.measure.*
import edu.wpi.first.units.measure.Velocity
import kotlin.math.*


//Unit Conversions

//Distance
inline val Distance.asInches: Double get() = `in`(Inches)
inline val Distance.asFeet: Double get() = `in`(Feet)
inline val Distance.asMeters: Double get() = `in`(Meters)
inline val Distance.asCentimeters: Double get() = `in`(Centimeters)
inline val Distance.asMillimeters: Double get() = `in`(Millimeter)

inline val Number.inches: Distance get() = Inches.of(this.toDouble())
inline val Number.feet: Distance get() = Feet.of(this.toDouble())
inline val Number.meters: Distance get() = Meters.of(this.toDouble())
inline val Number.centimeters: Distance get() = Centimeters.of(this.toDouble())
inline val Number.millimeters: Distance get() = Millimeters.of(this.toDouble())


//Angle
inline val Angle.asDegrees: Double get() = `in`(Degrees)
inline val Angle.asRotations: Double get() = `in`(Rotations)
inline val Angle.asRadians: Double get() = `in`(Radians)

inline val Number.degrees: Angle get() = Degrees.of(this.toDouble())
inline val Number.rotations: Angle get() = Rotations.of(this.toDouble())
inline val Number.radians: Angle get() = Radians.of(this.toDouble())


//Time
inline val Time.asSeconds: Double get() = `in`(Seconds)
inline val Time.asMinutes: Double get() = `in`(Minutes)
inline val Time.asMilliseconds: Double get() = `in`(Milliseconds)
inline val Time.asMicroseconds: Double get() = `in`(Microseconds)

inline val Number.seconds: Time get() = Seconds.of(this.toDouble())
inline val Number.milliseconds: Time get() = Milliseconds.of(this.toDouble())
inline val Number.microseconds: Time get() = Microseconds.of(this.toDouble())
inline val Number.minutes: Time get() = Minutes.of(this.toDouble())


//Linear Velocity
inline val LinearVelocity.asInchesPerSecond: Double get() = `in`(InchesPerSecond)
inline val LinearVelocity.asFeetPerSecond: Double get() = `in`(FeetPerSecond)
inline val LinearVelocity.asMetersPerSecond: Double get() = `in`(MetersPerSecond)

inline val Number.asInchesPerSecond: LinearVelocity get() = InchesPerSecond.of(this.toDouble())
inline val Number.feetPerSecond: LinearVelocity get() = FeetPerSecond.of(this.toDouble())
inline val Number.metersPerSecond: LinearVelocity get() = MetersPerSecond.of(this.toDouble())


//Angular Velocity
inline val AngularVelocity.asDegreesPerSecond: Double get() = `in`(DegreesPerSecond)
inline val AngularVelocity.asRotationsPerSecond: Double get() = `in`(RotationsPerSecond)
inline val AngularVelocity.asRPM: Double get() = `in`(RPM)
inline val AngularVelocity.asRadiansPerSecond: Double get() = `in`(RadiansPerSecond)

inline val Number.degreesPerSecond: AngularVelocity get() = DegreesPerSecond.of(this.toDouble())
inline val Number.rotationsPerSecond: AngularVelocity get() = RotationsPerSecond.of(this.toDouble())
inline val Number.rpm: AngularVelocity get() = RPM.of(this.toDouble())
inline val Number.radiansPerSecond: AngularVelocity get() = RadiansPerSecond.of(this.toDouble())


//Linear Acceleration
inline val LinearAcceleration.asFeetPerSecondPerSecond: Double get() = `in`(FeetPerSecondPerSecond)
inline val LinearAcceleration.asMetersPerSecondPerSecond: Double get() = `in`(MetersPerSecondPerSecond)
inline val LinearAcceleration.asGs: Double get() = `in`(Gs)

inline val Number.asInchesPerSecondPerSecond: LinearAcceleration get() = InchesPerSecond.per(Second).of(this.toDouble())
inline val Number.feetPerSecondPerSecond: LinearAcceleration get() = FeetPerSecondPerSecond.of(this.toDouble())
inline val Number.metersPerSecondPerSecond: LinearAcceleration get() = MetersPerSecondPerSecond.of(this.toDouble())
inline val Number.Gs: LinearAcceleration get() = Units.Gs.of(this.toDouble())


//Angular Acceleration
inline val AngularAcceleration.asDegreesPerSecondPerSecond: Double get() = `in`(DegreesPerSecondPerSecond)
inline val AngularAcceleration.asRotationsPerSecondPerSecond: Double get() = `in`(RotationsPerSecondPerSecond)
inline val AngularAcceleration.asRadiansPerSecondPerSecond: Double get() = `in`(RadiansPerSecondPerSecond)

inline val Number.degreesPerSecondPerSecond: AngularAcceleration get() = DegreesPerSecondPerSecond.of(this.toDouble())
inline val Number.rotationsPerSecondPerSecond: AngularAcceleration get() = RotationsPerSecondPerSecond.of(this.toDouble())
inline val Number.radiansPerSecondPerSecond: AngularAcceleration get() = RadiansPerSecondPerSecond.of(this.toDouble())


//Mass
inline val Mass.asKilograms: Double get() = `in`(Kilograms)
inline val Mass.asGrams: Double get() = `in`(Grams)
inline val Mass.asPounds: Double get() = `in`(Pounds)
inline val Mass.asOunces: Double get() = `in`(Ounces)

inline val Number.kilograms: Mass get() = Kilograms.of(this.toDouble())
inline val Number.grams: Mass get() = Grams.of(this.toDouble())
inline val Number.pounds: Mass get() = Pounds.of(this.toDouble())
inline val Number.ounces: Mass get() = Ounces.of(this.toDouble())


//The Force
inline val Force.asNewtons: Double get() = `in`(Newtons)
inline val Force.asOuncesForce: Double get() = `in`(OuncesForce)
inline val Force.asPoundsForce: Double get() = `in`(PoundsForce)

inline val Number.newtons: Force get() = Newtons.of(this.toDouble())
inline val Number.ouncesForce: Force get() = OuncesForce.of(this.toDouble())
inline val Number.poundsForce: Force get() = PoundsForce.of(this.toDouble())


//Torque
inline val Torque.asNewtonMeters: Double get() = `in`(NewtonMeters)
inline val Torque.asPoundFeet: Double get() = `in`(PoundFeet)
inline val Torque.asOunceInches: Double get() = `in`(OunceInches)

inline val Number.newtonMeters: Torque get() = NewtonMeters.of(this.toDouble())
inline val Number.poundFeet: Torque get() = PoundFeet.of(this.toDouble())
inline val Number.ounceInches: Torque get() = OunceInches.of(this.toDouble())


//MOI
inline val Number.kilogramSquareMeters: MomentOfInertia get() = KilogramSquareMeters.of(this.toDouble())


//Voltage
inline val Number.volts: Voltage get() = Volts.of(this.toDouble())

inline val Voltage.asVolts: Double get() = `in`(Volts)

//Current
inline val Current.asAmps: Double get() = `in`(Amps)

inline val Number.amps: Current get() = Amps.of(this.toDouble())


//Other
inline val Number.voltsPerSecond: Velocity<VoltageUnit> get() = Volts.per(Second).of(this.toDouble())

inline val Velocity<VoltageUnit>.asVoltsPerSecond: Double get() = `in`(Volts.per(Second))


//Formulas
fun LinearVelocity.toAngular(radius: Distance) = RadiansPerSecond.of(this.asMetersPerSecond / radius.asMeters)!!
fun AngularVelocity.toLinear(radius: Distance) = MetersPerSecond.of(this.asRadiansPerSecond * radius.asMeters)!!


/**Converts a [Number] in hertz into an equivalent [Time] unit.*/
fun Number.hertzToTime() = if (this == 0.0) 0.seconds else (1.0 / this.toDouble()).seconds

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





