
def norm(vector):
    return math.sqrt(sum(map(lambda coord: coord*coord, vector)))

def dot_product(vector1, vector2):
    if len(vector1) != len(vector2):
        raise Exception("Vector size mismatch")
    return sum(map(operator.mul, vector1, vector2))