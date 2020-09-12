from msp.data_structures.channels import Channel


if __name__ == "__main__":
    result = Channel.degree_to_value(180)
    if result != 1500:
        raise Exception("180 Degrees should equal 1500, instead got {0}".format(result))
    if Channel.degree_to_value(0) != 1001.4:
        raise Exception("0 Degrees should equal 1001.4, instead got {0}".format(result))
    if Channel.degree_to_value(360) != 1998.6:
        raise Exception("360 Degrees should equal 1998.6")

    result = Channel.percent_to_value(0)
    if  result != 1000:
        raise Exception("0% should equal 1000, instead got {0}".format(result))

    result = Channel.percent_to_value(50)
    if result != 1500:
        raise Exception("50% should equal 1500, instead got {0}".format(result))

    result = Channel.percent_to_value(75)
    if result != 1750:
        raise Exception("75% should equal 1750, instead got {0}".format(result))

