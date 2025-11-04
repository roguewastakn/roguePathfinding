--- Vector3.lua (Mock for standard Lua environment) ---

local Vector3 = {}
Vector3.__index = Vector3

function Vector3.new(x, y, z)
    local self = setmetatable({}, Vector3)
    self.X = x or 0
    self.Y = y or 0
    self.Z = z or 0
    return self
end

-- Operator Overloads (Simplified for necessary operations)

function Vector3:__add(other)
    return Vector3.new(self.X + other.X, self.Y + other.Y, self.Z + other.Z)
end

function Vector3:__sub(other)
    return Vector3.new(self.X - other.X, self.Y - other.Y, self.Z - other.Z)
end

function Vector3:__mul(scalar)
    return Vector3.new(self.X * scalar, self.Y * scalar, self.Z * scalar)
end

function Vector3:__div(scalar)
    return Vector3.new(self.X / scalar, self.Y / scalar, self.Z / scalar)
end

function Vector3:__tostring()
    return string.format("Vector3(%f, %f, %f)", self.X, self.Y, self.Z)
end

-- Properties and Methods

function Vector3:Dot(other)
    return self.X * other.X + self.Y * other.Y + self.Z * other.Z
end

function Vector3:MagnitudeSq()
    return self.X^2 + self.Y^2 + self.Z^2
end

function Vector3:Magnitude()
    return math.sqrt(self:MagnitudeSq())
end

function Vector3:Unit()
    local mag = self:Magnitude()
    if mag == 0 then
        return Vector3.new(0, 0, 0)
    else
        return self / mag
    end
end

-- Alias for Roblox compatibility
Vector3.unit = Vector3.Unit
Vector3.MagnitudeSq = Vector3.MagnitudeSq

return Vector3
