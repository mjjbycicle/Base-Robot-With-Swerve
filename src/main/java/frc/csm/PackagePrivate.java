package frc.csm;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Explicitly indicates that a type, method, or field is package-private. This
 * has absolutely no effect on compilation and runtime.
 */
@Retention(RetentionPolicy.SOURCE)
@Target({
        ElementType.TYPE,
        ElementType.METHOD,
        ElementType.CONSTRUCTOR,
        ElementType.FIELD
})
public @interface PackagePrivate {
}